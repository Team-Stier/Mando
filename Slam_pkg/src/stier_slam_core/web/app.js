(function (global) {
  'use strict';

  const whitespace = new Set([9, 10, 11, 12, 13, 32]);

  function asBytes(value) {
    if (value instanceof Uint8Array) return value;
    if (value instanceof ArrayBuffer) return new Uint8Array(value);
    throw new TypeError('PGM input must be an ArrayBuffer or Uint8Array');
  }

  function parsePgm(value) {
    const bytes = asBytes(value);
    let position = 0;
    const decoder = new TextDecoder('ascii', {fatal: true});

    function nextToken() {
      while (position < bytes.length) {
        if (whitespace.has(bytes[position])) {
          position += 1;
          continue;
        }
        if (bytes[position] === 35) {
          while (position < bytes.length && bytes[position] !== 10) position += 1;
          continue;
        }
        break;
      }
      if (position >= bytes.length) throw new Error('PGM header is truncated');
      const start = position;
      while (position < bytes.length && !whitespace.has(bytes[position]) && bytes[position] !== 35) {
        position += 1;
      }
      return decoder.decode(bytes.subarray(start, position));
    }

    function positiveInteger(token, label) {
      if (!/^[0-9]+$/.test(token)) throw new Error(`${label} must be an integer`);
      const number = Number(token);
      if (!Number.isSafeInteger(number) || number <= 0) throw new Error(`${label} is invalid`);
      return number;
    }

    const magic = nextToken();
    if (magic !== 'P2' && magic !== 'P5') throw new Error('Only P2 and P5 PGM are supported');
    const width = positiveInteger(nextToken(), 'PGM width');
    const height = positiveInteger(nextToken(), 'PGM height');
    const maxValue = positiveInteger(nextToken(), 'PGM max value');
    if (maxValue > 65535) throw new Error('PGM max value exceeds 65535');
    const sampleCount = width * height;
    if (!Number.isSafeInteger(sampleCount)) throw new Error('PGM dimensions are too large');
    const samples = new Uint16Array(sampleCount);

    if (magic === 'P2') {
      for (let index = 0; index < sampleCount; index += 1) {
        const sample = positiveIntegerAllowZero(nextToken(), 'P2 sample');
        if (sample > maxValue) throw new Error('P2 sample exceeds max value');
        samples[index] = sample;
      }
      skipWhitespaceAndComments();
      if (position !== bytes.length) throw new Error('P2 contains surplus samples');
    } else {
      if (position >= bytes.length || !whitespace.has(bytes[position])) {
        throw new Error('P5 header must end with whitespace');
      }
      if (bytes[position] === 13 && bytes[position + 1] === 10) position += 2;
      else position += 1;
      const bytesPerSample = maxValue < 256 ? 1 : 2;
      if (bytes.length - position !== sampleCount * bytesPerSample) {
        throw new Error('P5 raster length does not match dimensions');
      }
      for (let index = 0; index < sampleCount; index += 1) {
        const sample = bytesPerSample === 1
          ? bytes[position + index]
          : (bytes[position + index * 2] << 8) | bytes[position + index * 2 + 1];
        if (sample > maxValue) throw new Error('P5 sample exceeds max value');
        samples[index] = sample;
      }
    }

    return {magic, width, height, maxValue, samples};

    function positiveIntegerAllowZero(token, label) {
      if (!/^[0-9]+$/.test(token)) throw new Error(`${label} must be an integer`);
      const number = Number(token);
      if (!Number.isSafeInteger(number) || number < 0) throw new Error(`${label} is invalid`);
      return number;
    }

    function skipWhitespaceAndComments() {
      while (position < bytes.length) {
        if (whitespace.has(bytes[position])) {
          position += 1;
        } else if (bytes[position] === 35) {
          while (position < bytes.length && bytes[position] !== 10) position += 1;
        } else {
          break;
        }
      }
    }
  }

  function validateMapMetadata(map) {
    if (!map || !Number.isInteger(map.width) || map.width <= 0
        || !Number.isInteger(map.height) || map.height <= 0
        || !Number.isFinite(map.resolution) || map.resolution <= 0
        || !Array.isArray(map.origin) || map.origin.length !== 3
        || !map.origin.every(Number.isFinite) || map.origin[2] !== 0) {
      throw new Error('Map metadata is invalid');
    }
  }

  function clamp(value, minimum, maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  function cleanNumber(value) {
    return Number(value.toPrecision(15));
  }

  function pixelToMap(map, pixelX, pixelY, shouldClamp) {
    validateMapMetadata(map);
    if (!Number.isFinite(pixelX) || !Number.isFinite(pixelY)) throw new Error('Pixel is invalid');
    let x = pixelX;
    let y = pixelY;
    if (shouldClamp) {
      x = clamp(x, 0, map.width);
      y = clamp(y, 0, map.height);
    } else if (x < 0 || x > map.width || y < 0 || y > map.height) {
      throw new Error('Pixel is outside the map');
    }
    return [
      cleanNumber(map.origin[0] + x * map.resolution),
      cleanNumber(map.origin[1] + (map.height - y) * map.resolution),
    ];
  }

  function mapToPixel(map, mapX, mapY, shouldClamp) {
    validateMapMetadata(map);
    if (!Number.isFinite(mapX) || !Number.isFinite(mapY)) throw new Error('Coordinate is invalid');
    let x = (mapX - map.origin[0]) / map.resolution;
    let y = map.height - (mapY - map.origin[1]) / map.resolution;
    if (shouldClamp) {
      x = clamp(x, 0, map.width);
      y = clamp(y, 0, map.height);
    } else if (x < 0 || x > map.width || y < 0 || y > map.height) {
      throw new Error('Coordinate is outside the map');
    }
    return [cleanNumber(x), cleanNumber(y)];
  }

  function canEditCanvas(busy, workspace) {
    return !busy && Boolean(workspace);
  }

  async function sha256Hex(value) {
    const bytes = asBytes(value);
    if (!global.crypto || !global.crypto.subtle) {
      throw new Error('이 브라우저는 지도 SHA-256 검증을 지원하지 않습니다.');
    }
    const digest = await global.crypto.subtle.digest(
      'SHA-256', bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength),
    );
    return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, '0')).join('');
  }

  global.StierMapEditorCore = {
    parsePgm, pixelToMap, mapToPixel, canEditCanvas, sha256Hex,
  };

  if (typeof document === 'undefined') return;

  const elements = {
    canvas: document.getElementById('mapCanvas'),
    viewport: document.getElementById('canvasViewport'),
    status: document.getElementById('status'),
    workspaceBadge: document.getElementById('workspaceBadge'),
    revisionBadge: document.getElementById('revisionBadge'),
    modeStatus: document.getElementById('modeStatus'),
    cursorStatus: document.getElementById('cursorStatus'),
    draftHelp: document.getElementById('draftHelp'),
    semanticLabel: document.getElementById('semanticLabel'),
    finishShape: document.getElementById('finishShape'),
    cancelShape: document.getElementById('cancelShape'),
    undo: document.getElementById('undo'),
    clearAll: document.getElementById('clearAll'),
    validate: document.getElementById('validate'),
    save: document.getElementById('save'),
    release: document.getElementById('release'),
    releaseId: document.getElementById('releaseId'),
    activateRelease: document.getElementById('activateRelease'),
    releaseHashes: document.getElementById('releaseHashes'),
    resetView: document.getElementById('resetView'),
  };

  const context = elements.canvas.getContext('2d');
  const state = {
    workspace: null,
    pgm: null,
    baseImage: null,
    mode: 'pan',
    draft: [],
    preview: null,
    history: [],
    dirty: false,
    busy: false,
    zoom: 1,
    offsetX: 0,
    offsetY: 0,
    pan: null,
  };

  function setStatus(message, kind) {
    elements.status.textContent = message;
    elements.status.className = `status${kind ? ` ${kind}` : ''}`;
  }

  function setBusy(message) {
    state.busy = true;
    setStatus(message, 'busy');
    for (const control of document.querySelectorAll('button, input, select')) {
      control.disabled = true;
    }
  }

  function clearBusy() {
    state.busy = false;
    for (const control of document.querySelectorAll('button, input, select')) {
      control.disabled = false;
    }
  }

  function updateBadges() {
    if (!state.workspace) return;
    elements.workspaceBadge.textContent = `${state.workspace.workspace_id} · ${state.workspace.capture_id}`;
    elements.revisionBadge.textContent = `revision ${state.workspace.revision.slice(0, 12)}${state.dirty ? ' · 저장 안 됨' : ''}`;
  }

  function applyView() {
    elements.canvas.style.transform = `translate(${state.offsetX}px, ${state.offsetY}px) scale(${state.zoom})`;
  }

  function resetView() {
    if (!state.pgm) return;
    const availableWidth = Math.max(1, elements.viewport.clientWidth - 32);
    const availableHeight = Math.max(1, elements.viewport.clientHeight - 32);
    state.zoom = clamp(Math.min(availableWidth / state.pgm.width, availableHeight / state.pgm.height), 0.05, 20);
    state.offsetX = (elements.viewport.clientWidth - state.pgm.width * state.zoom) / 2;
    state.offsetY = (elements.viewport.clientHeight - state.pgm.height * state.zoom) / 2;
    applyView();
  }

  function imagePoint(event) {
    const rect = elements.canvas.getBoundingClientRect();
    return [
      clamp((event.clientX - rect.left) * state.pgm.width / rect.width, 0, state.pgm.width),
      clamp((event.clientY - rect.top) * state.pgm.height / rect.height, 0, state.pgm.height),
    ];
  }

  function roundedMapPoint(event) {
    const point = imagePoint(event);
    return pixelToMap(state.workspace.map, point[0], point[1], true)
      .map((value) => Number(value.toFixed(6)));
  }

  function makeBaseImage(pgm) {
    const image = context.createImageData(pgm.width, pgm.height);
    for (let index = 0; index < pgm.samples.length; index += 1) {
      const value = Math.round(pgm.samples[index] * 255 / pgm.maxValue);
      const offset = index * 4;
      image.data[offset] = value;
      image.data[offset + 1] = value;
      image.data[offset + 2] = value;
      image.data[offset + 3] = 255;
    }
    return image;
  }

  function render() {
    if (!state.baseImage || !state.workspace) return;
    context.putImageData(state.baseImage, 0, 0);
    const lineWidth = Math.max(1, 2 / state.zoom);
    for (const operation of state.workspace.static_overlay.operations) {
      drawPath(operation.polygon, true);
      context.fillStyle = operation.operation === 'occupied'
        ? 'rgba(255, 70, 70, 0.36)' : 'rgba(56, 217, 169, 0.32)';
      context.strokeStyle = operation.operation === 'occupied' ? '#ff5555' : '#38d9a9';
      context.lineWidth = lineWidth;
      context.fill();
      context.stroke();
    }
    for (const feature of state.workspace.semantic_layer.features) drawSemantic(feature, lineWidth);
    if (state.draft.length) {
      drawPath(state.draft, false);
      context.strokeStyle = '#f6c453';
      context.lineWidth = lineWidth;
      context.setLineDash([5 / state.zoom, 4 / state.zoom]);
      context.stroke();
      context.setLineDash([]);
      for (const point of state.draft) drawPoint(point, '#f6c453', 3 / state.zoom);
    }
    if (state.preview && state.draft.length) {
      const previous = mapToPixel(state.workspace.map, ...state.draft[state.draft.length - 1], true);
      const preview = mapToPixel(state.workspace.map, ...state.preview, true);
      context.beginPath();
      context.moveTo(previous[0], previous[1]);
      context.lineTo(preview[0], preview[1]);
      context.strokeStyle = '#f6c453';
      context.lineWidth = lineWidth;
      context.stroke();
    }
  }

  function drawPath(points, close) {
    context.beginPath();
    points.forEach((point, index) => {
      const pixel = mapToPixel(state.workspace.map, point[0], point[1], true);
      if (index === 0) context.moveTo(pixel[0], pixel[1]);
      else context.lineTo(pixel[0], pixel[1]);
    });
    if (close) context.closePath();
  }

  function drawPoint(point, color, radius) {
    const pixel = mapToPixel(state.workspace.map, point[0], point[1], true);
    context.beginPath();
    context.arc(pixel[0], pixel[1], Math.max(1.5, radius), 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
  }

  function drawSemantic(feature, lineWidth) {
    const colors = {
      lane_center: '#55aaff', stop_line: '#ffbd55', intersection: '#d678ff', course_boundary: '#ffffff',
    };
    const color = colors[feature.label] || '#ffffff';
    if (feature.geometry.type === 'point') {
      drawPoint(feature.geometry.coordinates, color, 4 / state.zoom);
      return;
    }
    drawPath(feature.geometry.coordinates, false);
    context.strokeStyle = color;
    context.lineWidth = lineWidth;
    context.stroke();
  }

  function snapshotLayers() {
    return JSON.stringify({
      static_overlay: state.workspace.static_overlay,
      semantic_layer: state.workspace.semantic_layer,
    });
  }

  function pushHistory() {
    state.history.push(snapshotLayers());
    if (state.history.length > 100) state.history.shift();
  }

  function restoreLayers(serialized) {
    const layers = JSON.parse(serialized);
    state.workspace.static_overlay = layers.static_overlay;
    state.workspace.semantic_layer = layers.semantic_layer;
    state.dirty = true;
    updateBadges();
    render();
  }

  function selectMode(mode) {
    if (state.busy) return;
    state.mode = mode;
    state.draft = [];
    state.preview = null;
    document.querySelectorAll('[data-mode]').forEach((button) => {
      button.classList.toggle('active', button.dataset.mode === mode);
    });
    const help = {
      pan: '드래그로 지도를 이동하고 휠로 확대·축소합니다.',
      occupied: '장애물 다각형 꼭짓점을 찍고 도형 완료를 누르세요.',
      free: '복원할 빈 공간 다각형 꼭짓점을 찍고 도형 완료를 누르세요.',
      'semantic-point': 'intersection 위치를 한 번 클릭하세요.',
      'semantic-polyline': '의미 선의 점을 찍고 도형 완료를 누르세요.',
    };
    elements.draftHelp.textContent = help[mode];
    elements.modeStatus.textContent = `mode: ${mode}`;
    render();
  }

  function finishShape() {
    if (!canEditCanvas(state.busy, state.workspace)) return;
    if (state.mode === 'occupied' || state.mode === 'free') {
      if (state.draft.length < 3) throw new Error('다각형은 점이 3개 이상 필요합니다.');
      pushHistory();
      state.workspace.static_overlay.operations.push({
        operation: state.mode,
        polygon: state.draft.map((point) => [...point]),
      });
    } else if (state.mode === 'semantic-polyline') {
      if (state.draft.length < 2) throw new Error('의미 선은 서로 다른 점이 2개 이상 필요합니다.');
      const label = elements.semanticLabel.value;
      if (label === 'intersection') throw new Error('intersection은 점 도구를 사용하세요.');
      pushHistory();
      state.workspace.semantic_layer.features.push({
        label,
        geometry: {type: 'polyline', coordinates: state.draft.map((point) => [...point])},
      });
    } else {
      return;
    }
    state.draft = [];
    state.preview = null;
    state.dirty = true;
    updateBadges();
    render();
    setStatus('편집이 메모리에 추가되었습니다. 검증 후 저장하세요.', 'success');
  }

  function validateClient() {
    const overlay = state.workspace.static_overlay;
    const semantic = state.workspace.semantic_layer;
    if (overlay.schema_version !== 1 || !Array.isArray(overlay.operations)) throw new Error('Overlay schema가 잘못되었습니다.');
    for (const operation of overlay.operations) {
      if (!['occupied', 'free'].includes(operation.operation) || !Array.isArray(operation.polygon)
          || operation.polygon.length < 3) throw new Error('Overlay 다각형이 잘못되었습니다.');
    }
    if (semantic.schema_version !== 1 || !Array.isArray(semantic.features)) throw new Error('Semantic schema가 잘못되었습니다.');
    for (const feature of semantic.features) {
      if (!['lane_center', 'stop_line', 'intersection', 'course_boundary'].includes(feature.label)) {
        throw new Error('지원하지 않는 semantic label입니다.');
      }
    }
    return true;
  }

  function requireFinishedDraft() {
    if (state.draft.length) {
      throw new Error('화면의 노란 draft를 도형 완료하거나 취소한 뒤 저장·release 하세요.');
    }
  }

  async function fetchJson(url, options) {
    const response = await fetch(url, options);
    let document;
    try { document = await response.json(); } catch (_) { throw new Error(`HTTP ${response.status}: JSON 응답이 아닙니다.`); }
    if (!response.ok) {
      const message = document.error && document.error.message ? document.error.message : `HTTP ${response.status}`;
      const error = new Error(message);
      error.code = document.error && document.error.code;
      throw error;
    }
    return document;
  }

  async function saveWorkspace() {
    requireFinishedDraft();
    validateClient();
    const ownsBusy = !state.busy;
    if (ownsBusy) {
      setBusy('서버에서 geometry와 map bounds를 검증하고 두 layer를 원자적으로 저장하는 중입니다.');
    }
    try {
      const updated = await fetchJson('/api/workspace', {
        method: 'PUT',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          schema_version: 1,
          expected_revision: state.workspace.revision,
          static_overlay: state.workspace.static_overlay,
          semantic_layer: state.workspace.semantic_layer,
        }),
      });
      state.workspace = updated;
      state.dirty = false;
      state.history = [];
      updateBadges();
      render();
      if (updated.error && updated.error.code === 'workspace_durability_indeterminate') {
        setStatus(
          '편집 revision은 교체됐지만 영구 저장 여부가 불명확합니다. 확인하거나 이 revision을 다시 저장하세요.',
          'error',
        );
      } else {
        setStatus('검증과 원자적 저장이 완료되었습니다.', 'success');
      }
      return updated;
    } catch (error) {
      if (error.code === 'revision_conflict') {
        throw new Error('다른 편집이 먼저 저장되었습니다. 페이지를 새로고침해 최신 revision을 확인하세요.');
      }
      throw error;
    } finally {
      if (ownsBusy) clearBusy();
    }
  }

  async function createRelease() {
    if (state.busy) return;
    requireFinishedDraft();
    const releaseId = elements.releaseId.value.trim();
    const activate = elements.activateRelease.checked;
    elements.releaseHashes.textContent = '';
    if (!/^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/.test(releaseId)) {
      throw new Error('Release ID는 영문·숫자로 시작하는 1–64자 안전 식별자여야 합니다.');
    }
    setBusy('편집을 저장하고 불변 release hash를 검증하는 중입니다.');
    try {
      if (state.dirty) await saveWorkspace();
      let summary;
      try {
        summary = await fetchJson('/api/release', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            schema_version: 1,
            expected_revision: state.workspace.revision,
            release_id: releaseId,
            activate,
          }),
        });
      } catch (error) {
        if (error && error.code) throw error;
        throw new Error(
          `${releaseId} release 요청 결과를 확인할 수 없습니다. 생성되었을 수 있으므로 같은 ID로 안전하게 재시도하세요.`,
        );
      }
      const activationLabel = summary.activated === true
        ? 'true' : summary.activated === 'indeterminate' ? 'indeterminate' : 'false';
      elements.releaseHashes.textContent = [
        `release: ${summary.release_id}`,
        `activated: ${activationLabel}`,
        `manifest: ${summary.manifest_sha256}`,
        ...Object.entries(summary.artifact_hashes).map(([name, digest]) => `${name}: ${digest}`),
      ].join('\n');
      if (summary.error && summary.error.code === 'release_publication_indeterminate') {
        setStatus(
          `${summary.release_id} release 파일은 생성됐지만 영구 저장 여부가 불명확합니다. 같은 ID로 확인·재시도하세요.`,
          'error',
        );
      } else if (summary.activated === 'indeterminate') {
        setStatus(
          `${summary.release_id} release는 생성되었고 active pointer도 일치하지만 영구 반영 여부가 불명확합니다. 확인하거나 같은 ID로 재시도하세요.`,
          'error',
        );
      } else if (activate && summary.activated !== true) {
        const detail = summary.error && summary.error.message
          ? ` (${summary.error.message})` : '';
        setStatus(
          `${summary.release_id} release는 생성되었지만 활성화에 실패했습니다${detail}. 같은 ID로 다시 활성화를 시도하세요.`,
          'error',
        );
      } else if (summary.activated === true) {
        setStatus(
          `${summary.release_id} 생성 및 활성화 완료. 다음 localization 시작부터 적용됩니다.`,
          'success',
        );
      } else {
        setStatus(
          `${summary.release_id} 생성 완료(비활성). 활성화 전까지 현재 release가 유지됩니다.`,
          'success',
        );
      }
    } finally {
      clearBusy();
    }
  }

  function handleError(error) {
    setStatus(error && error.message ? error.message : String(error), 'error');
  }

  async function boot() {
    setBusy('검증된 workspace와 base PGM을 불러오는 중입니다.');
    try {
      const [workspace, pgmResponse] = await Promise.all([
        fetchJson('/api/workspace'), fetch('/api/base.pgm'),
      ]);
      if (!pgmResponse.ok) throw new Error(`Base PGM HTTP ${pgmResponse.status}`);
      const pgmBuffer = await pgmResponse.arrayBuffer();
      const actualImageSha256 = await sha256Hex(pgmBuffer);
      if (typeof workspace.map.image_sha256 !== 'string'
          || !/^[0-9a-f]{64}$/.test(workspace.map.image_sha256)
          || actualImageSha256 !== workspace.map.image_sha256) {
        throw new Error('Base PGM SHA-256이 workspace identity와 다릅니다. 편집을 중단합니다.');
      }
      const pgm = parsePgm(pgmBuffer);
      if (pgm.width !== workspace.map.width || pgm.height !== workspace.map.height) {
        throw new Error('PGM 크기와 workspace metadata가 다릅니다.');
      }
      state.workspace = workspace;
      state.pgm = pgm;
      elements.canvas.width = pgm.width;
      elements.canvas.height = pgm.height;
      state.baseImage = makeBaseImage(pgm);
      render();
      resetView();
      updateBadges();
      setStatus('지도 준비 완료. 편집은 map-frame meter로 저장됩니다.', 'success');
    } catch (error) {
      handleError(error);
    } finally {
      clearBusy();
    }
  }

  document.querySelectorAll('[data-mode]').forEach((button) => {
    button.addEventListener('click', () => selectMode(button.dataset.mode));
  });
  elements.finishShape.addEventListener('click', () => { try { finishShape(); } catch (error) { handleError(error); } });
  elements.cancelShape.addEventListener('click', () => {
    if (state.busy) return;
    state.draft = [];
    state.preview = null;
    render();
  });
  elements.undo.addEventListener('click', () => {
    if (state.busy) return;
    const previous = state.history.pop();
    if (previous) restoreLayers(previous);
    else setStatus('실행 취소할 편집이 없습니다.');
  });
  elements.clearAll.addEventListener('click', () => {
    if (state.busy || !state.workspace
        || !global.confirm('모든 occupancy와 semantic 편집을 지울까요?')) return;
    pushHistory();
    state.workspace.static_overlay = {schema_version: 1, operations: []};
    state.workspace.semantic_layer = {schema_version: 1, features: []};
    state.draft = [];
    state.dirty = true;
    updateBadges();
    render();
  });
  elements.validate.addEventListener('click', () => {
    if (state.busy) return;
    try { validateClient(); setStatus('브라우저 형식 검증 통과. 저장 시 서버가 geometry와 bounds를 다시 검증합니다.', 'success'); }
    catch (error) { handleError(error); }
  });
  elements.save.addEventListener('click', () => {
    if (!state.busy) saveWorkspace().catch(handleError);
  });
  elements.release.addEventListener('click', () => {
    if (!state.busy) createRelease().catch(handleError);
  });
  elements.resetView.addEventListener('click', () => {
    if (!state.busy) resetView();
  });

  elements.canvas.addEventListener('pointerdown', (event) => {
    if (!canEditCanvas(state.busy, state.workspace)) return;
    if (state.mode === 'pan') {
      state.pan = {x: event.clientX, y: event.clientY, offsetX: state.offsetX, offsetY: state.offsetY};
      elements.canvas.setPointerCapture(event.pointerId);
      return;
    }
    const point = roundedMapPoint(event);
    if (state.mode === 'semantic-point') {
      pushHistory();
      state.workspace.semantic_layer.features.push({
        label: 'intersection', geometry: {type: 'point', coordinates: point},
      });
      elements.semanticLabel.value = 'intersection';
      state.dirty = true;
      updateBadges();
      render();
      return;
    }
    if (!state.draft.some((existing) => existing[0] === point[0] && existing[1] === point[1])) {
      state.draft.push(point);
      render();
    }
  });
  elements.canvas.addEventListener('pointermove', (event) => {
    if (!canEditCanvas(state.busy, state.workspace)) return;
    const point = roundedMapPoint(event);
    elements.cursorStatus.textContent = `map: ${point[0].toFixed(3)}, ${point[1].toFixed(3)} m`;
    if (state.pan) {
      state.offsetX = state.pan.offsetX + event.clientX - state.pan.x;
      state.offsetY = state.pan.offsetY + event.clientY - state.pan.y;
      applyView();
    } else if (state.mode !== 'pan') {
      state.preview = point;
      render();
    }
  });
  elements.canvas.addEventListener('pointerup', () => { state.pan = null; });
  elements.canvas.addEventListener('pointercancel', () => { state.pan = null; });
  elements.canvas.addEventListener('dblclick', (event) => {
    event.preventDefault();
    try { finishShape(); } catch (error) { handleError(error); }
  });
  elements.viewport.addEventListener('wheel', (event) => {
    if (state.busy || !state.pgm) return;
    event.preventDefault();
    const rect = elements.viewport.getBoundingClientRect();
    const localX = (event.clientX - rect.left - state.offsetX) / state.zoom;
    const localY = (event.clientY - rect.top - state.offsetY) / state.zoom;
    const nextZoom = clamp(state.zoom * Math.exp(-event.deltaY * 0.001), 0.05, 40);
    state.offsetX = event.clientX - rect.left - localX * nextZoom;
    state.offsetY = event.clientY - rect.top - localY * nextZoom;
    state.zoom = nextZoom;
    applyView();
    render();
  }, {passive: false});
  global.addEventListener('resize', () => { if (state.pgm) resetView(); });

  boot();
}(typeof window !== 'undefined' ? window : globalThis));
