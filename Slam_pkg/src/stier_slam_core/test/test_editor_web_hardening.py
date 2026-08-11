import hashlib
import json
from pathlib import Path
import subprocess
import unittest


APP_PATH = Path(__file__).parents[1] / "web" / "app.js"
ORIGINAL_PGM = b"P5\n2 2\n255\n" + bytes((0, 64, 128, 255))
TAMPERED_PGM = b"P5\n2 2\n255\n" + bytes((1, 64, 128, 255))


def browser_harness(pgm_bytes, advertised_sha256, scenario):
    preamble = r"""
const assert = require('assert');
const appPath = process.argv[1];
const pgmBytes = Uint8Array.from(PGM_BYTES);

class TestElement {
  constructor(id, tagName) {
    this.id = id;
    this.tagName = tagName.toUpperCase();
    this.listeners = {};
    this.attributes = {};
    this.className = '';
    this.textContent = '';
    this.value = '';
    this.checked = false;
    this.disabled = false;
    this.dataset = {};
    this.style = {};
    this.clientWidth = 640;
    this.clientHeight = 480;
    this.classList = {toggle() {}};
  }
  addEventListener(name, listener) {
    if (!this.listeners[name]) this.listeners[name] = [];
    this.listeners[name].push(listener);
  }
  emit(name, event = {}) {
    for (const listener of this.listeners[name] || []) listener(event);
  }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  removeAttribute(name) { delete this.attributes[name]; }
  getAttribute(name) { return this.attributes[name] || null; }
}

const buttonIds = new Set([
  'finishShape', 'cancelShape', 'undo', 'clearAll', 'validate', 'save',
  'release', 'resetView', 'modeOccupied',
]);
const inputIds = new Set(['releaseId', 'activateRelease']);
const selectIds = new Set(['semanticLabel']);
const ids = [
  'mapCanvas', 'canvasViewport', 'status', 'workspaceBadge', 'revisionBadge',
  'modeStatus', 'cursorStatus', 'draftHelp', 'semanticLabel', 'finishShape',
  'cancelShape', 'undo', 'clearAll', 'validate', 'save', 'release', 'releaseId',
  'activateRelease', 'releaseHashes', 'resetView', 'modeOccupied',
];
const elements = {};
for (const id of ids) {
  const tag = buttonIds.has(id) ? 'button'
    : inputIds.has(id) ? 'input'
      : selectIds.has(id) ? 'select' : id === 'mapCanvas' ? 'canvas' : 'div';
  elements[id] = new TestElement(id, tag);
}
elements.modeOccupied.dataset.mode = 'occupied';
elements.semanticLabel.value = 'lane_center';

const drawingContext = {
  createImageData(width, height) {
    return {data: new Uint8ClampedArray(width * height * 4)};
  },
  putImageData() {}, beginPath() {}, moveTo() {}, lineTo() {}, closePath() {},
  fill() {}, stroke() {}, arc() {}, setLineDash() {},
};
elements.mapCanvas.getContext = () => drawingContext;
elements.mapCanvas.getBoundingClientRect = () => ({left: 0, top: 0, width: 2, height: 2});
elements.mapCanvas.setPointerCapture = () => {};

globalThis.document = {
  getElementById(id) { return elements[id]; },
  querySelectorAll(selector) {
    if (selector === '[data-mode]') return [elements.modeOccupied];
    const normalized = selector.replace(/\s/g, '');
    if (normalized === 'button') return ids.filter((id) => buttonIds.has(id)).map((id) => elements[id]);
    if (normalized === 'button,input,select') {
      return ids.filter((id) => buttonIds.has(id) || inputIds.has(id) || selectIds.has(id))
        .map((id) => elements[id]);
    }
    return [];
  },
};
globalThis.addEventListener = () => {};
globalThis.confirm = () => true;

const workspace = {
  schema_version: 1,
  workspace_id: 'workspace',
  capture_id: 'capture',
  revision: 'a'.repeat(64),
  map: {
    width: 2,
    height: 2,
    resolution: 0.1,
    origin: [0.0, 0.0, 0.0],
    image_url: '/api/base.pgm',
    image_sha256: ADVERTISED_SHA256,
  },
  static_overlay: {schema_version: 1, operations: []},
  semantic_layer: {schema_version: 1, features: []},
};
const savedWorkspace = JSON.parse(JSON.stringify(workspace));
savedWorkspace.revision = 'b'.repeat(64);

function jsonResponse(document, status = 200) {
  return {
    ok: status >= 200 && status < 300,
    status,
    headers: {get() { return null; }},
    async json() { return JSON.parse(JSON.stringify(document)); },
  };
}
function pgmResponse() {
  return {
    ok: true,
    status: 200,
    headers: {get() { return null; }},
    async arrayBuffer() {
      return pgmBytes.buffer.slice(pgmBytes.byteOffset, pgmBytes.byteOffset + pgmBytes.byteLength);
    },
  };
}

let holdSave = false;
let resolveSave = null;
let putCount = 0;
const posts = [];
globalThis.fetch = (url, options = {}) => {
  if (url === '/api/workspace' && !options.method) return Promise.resolve(jsonResponse(workspace));
  if (url.startsWith('/api/base.pgm')) return Promise.resolve(pgmResponse());
  if (url === '/api/workspace' && options.method === 'PUT') {
    putCount += 1;
    if (holdSave) return new Promise((resolve) => { resolveSave = resolve; });
    return Promise.resolve(jsonResponse(savedWorkspace));
  }
  if (url === '/api/release' && options.method === 'POST') {
    posts.push(JSON.parse(options.body));
    return Promise.resolve(jsonResponse({
      schema_version: 1,
      release_id: posts[posts.length - 1].release_id,
      activated: posts[posts.length - 1].activate,
      manifest_sha256: 'c'.repeat(64),
      artifact_hashes: {
        'map.pgm': 'd'.repeat(64),
        'map.yaml': 'e'.repeat(64),
        'static_overlay.json': 'f'.repeat(64),
        'semantic_layer.json': '0'.repeat(64),
      },
    }, 201));
  }
  throw new Error(`unexpected fetch ${options.method || 'GET'} ${url}`);
};

function immediate() { return new Promise((resolve) => setImmediate(resolve)); }
async function waitFor(predicate, label) {
  for (let attempt = 0; attempt < 50; attempt += 1) {
    if (predicate()) return;
    await immediate();
  }
  throw new Error(`timed out waiting for ${label}`);
}

(async () => {
  require(appPath);
  await waitFor(() => !elements.status.className.includes('busy'), 'editor boot');
SCENARIO
})().catch((error) => {
  console.error(error.stack || String(error));
  process.exitCode = 1;
});
"""
    return (preamble
            .replace("PGM_BYTES", json.dumps(list(pgm_bytes)))
            .replace("ADVERTISED_SHA256", json.dumps(advertised_sha256))
            .replace("SCENARIO", scenario))


class EditorWebHardeningRedTest(unittest.TestCase):
    def run_browser(self, pgm_bytes, advertised_sha256, scenario):
        result = subprocess.run(
            ["node", "-e", browser_harness(pgm_bytes, advertised_sha256, scenario), str(APP_PATH)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_release_uses_the_form_snapshot_from_the_original_click(self):
        """Edits to release controls during an awaited save must not alter click intent."""
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  elements.clearAll.emit('click');
  elements.releaseId.value = 'clicked-release';
  elements.activateRelease.checked = true;
  holdSave = true;
  elements.release.emit('click');
  await waitFor(() => putCount === 1 && resolveSave !== null, 'workspace save');

  elements.releaseId.value = 'changed-while-saving';
  elements.activateRelease.checked = false;
  resolveSave(jsonResponse(savedWorkspace));
  await waitFor(() => posts.length === 1, 'release request');

  assert.strictEqual(posts[0].release_id, 'clicked-release');
  assert.strictEqual(posts[0].activate, true);
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_save_and_release_reject_visible_unfinished_draft(self):
        """A visible draft must not be silently omitted from saved or released layers."""
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  elements.modeOccupied.emit('click');
  elements.mapCanvas.emit('pointerdown', {clientX: 0.5, clientY: 0.5, pointerId: 1});

  elements.save.emit('click');
  await waitFor(() => elements.status.className.includes('error'), 'draft save rejection');
  assert(elements.status.textContent.includes('도형 완료'));
  assert.strictEqual(putCount, 0);

  elements.releaseId.value = 'unfinished-draft';
  elements.release.emit('click');
  await waitFor(() => elements.status.className.includes('error'), 'draft release rejection');
  assert(elements.status.textContent.includes('도형 완료'));
  assert.strictEqual(posts.length, 0);
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_busy_release_locks_buttons_inputs_and_selects(self):
        """Busy state must freeze every control that contributes to a release request."""
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  elements.clearAll.emit('click');
  elements.releaseId.value = 'busy-release';
  elements.activateRelease.checked = true;
  holdSave = true;
  elements.release.emit('click');
  await waitFor(() => putCount === 1 && resolveSave !== null, 'workspace save');

  assert.strictEqual(elements.release.disabled, true, 'release button must be disabled');
  assert.strictEqual(elements.releaseId.disabled, true, 'release ID must be disabled');
  assert.strictEqual(elements.activateRelease.disabled, true, 'activate checkbox must be disabled');
  assert.strictEqual(elements.semanticLabel.disabled, true, 'semantic selector must be disabled');
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_boot_rejects_same_size_pgm_with_wrong_sha256(self):
        """Matching dimensions cannot substitute for WebCrypto verification of image identity."""
        scenario = r"""
  assert(
    elements.status.className.includes('error'),
    `tampered PGM was accepted: ${elements.status.textContent}`,
  );
"""
        self.run_browser(
            TAMPERED_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_invalid_release_id_clears_old_hashes_without_saving(self):
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  elements.clearAll.emit('click');
  elements.releaseHashes.textContent = 'stale release hashes';
  elements.releaseId.value = '../invalid';
  elements.release.emit('click');
  await waitFor(() => elements.status.className.includes('error'), 'invalid release');

  assert.strictEqual(putCount, 0, 'invalid release ID must be checked before save');
  assert.strictEqual(posts.length, 0);
  assert.strictEqual(elements.releaseHashes.textContent, '');
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_partial_release_shows_hashes_and_activation_failure_guidance(self):
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  const normalFetch = globalThis.fetch;
  globalThis.fetch = (url, options = {}) => {
    if (url === '/api/release' && options.method === 'POST') {
      const request = JSON.parse(options.body);
      posts.push(request);
      return Promise.resolve(jsonResponse({
        schema_version: 1,
        release_id: request.release_id,
        activated: false,
        manifest_sha256: 'c'.repeat(64),
        artifact_hashes: {
          'map.pgm': 'd'.repeat(64),
          'map.yaml': 'e'.repeat(64),
          'static_overlay.json': 'f'.repeat(64),
          'semantic_layer.json': '0'.repeat(64),
        },
        error: {code: 'activation_failed', message: 'injected activation failure'},
      }, 201));
    }
    return normalFetch(url, options);
  };
  elements.releaseId.value = 'partial-release';
  elements.activateRelease.checked = true;
  elements.release.emit('click');
  await waitFor(() => posts.length === 1, 'partial release request');
  await waitFor(() => elements.status.className.includes('error'), 'partial guidance');

  assert(elements.releaseHashes.textContent.includes('activated: false'));
  assert(elements.releaseHashes.textContent.includes('manifest: ' + 'c'.repeat(64)));
  assert(elements.status.textContent.includes('release는 생성되었지만 활성화에 실패'));
  assert(elements.status.textContent.includes('같은 ID로 다시 활성화'));
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_release_publication_ambiguity_preserves_hashes_and_same_id_guidance(self):
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  const normalFetch = globalThis.fetch;
  globalThis.fetch = (url, options = {}) => {
    if (url === '/api/release' && options.method === 'POST') {
      const request = JSON.parse(options.body);
      posts.push(request);
      return Promise.resolve(jsonResponse({
        schema_version: 1,
        release_id: request.release_id,
        activated: false,
        manifest_sha256: 'c'.repeat(64),
        artifact_hashes: {
          'map.pgm': 'd'.repeat(64),
          'map.yaml': 'e'.repeat(64),
          'static_overlay.json': 'f'.repeat(64),
          'semantic_layer.json': '0'.repeat(64),
        },
        error: {
          code: 'release_publication_indeterminate',
          message: 'injected release publication ambiguity',
        },
      }, 201));
    }
    return normalFetch(url, options);
  };
  elements.releaseId.value = 'publication-ambiguous';
  elements.activateRelease.checked = true;
  elements.release.emit('click');
  await waitFor(() => posts.length === 1, 'ambiguous publication request');
  await waitFor(() => elements.status.className.includes('error'), 'publication guidance');

  assert(elements.releaseHashes.textContent.includes('release: publication-ambiguous'));
  assert(elements.releaseHashes.textContent.includes('manifest: ' + 'c'.repeat(64)));
  assert(elements.status.textContent.includes('영구 저장 여부가 불명확'));
  assert(elements.status.textContent.includes('같은 ID'));
  assert(elements.status.textContent.includes('재시도'));
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_release_response_loss_reports_unknown_outcome_and_same_id_retry(self):
        """A lost POST response must not be presented as proof no release was created."""
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  const normalFetch = globalThis.fetch;
  globalThis.fetch = (url, options = {}) => {
    if (url === '/api/release' && options.method === 'POST') {
      posts.push(JSON.parse(options.body));
      return Promise.reject(new TypeError('connection lost after upload'));
    }
    return normalFetch(url, options);
  };
  elements.releaseId.value = 'ambiguous-release';
  elements.activateRelease.checked = true;
  elements.release.emit('click');
  await waitFor(() => posts.length === 1, 'ambiguous release request');
  await waitFor(() => elements.status.className.includes('error'), 'unknown outcome guidance');

  assert(elements.status.textContent.includes('ambiguous-release'));
  assert(elements.status.textContent.includes('결과를 확인할 수 없습니다'));
  assert(elements.status.textContent.includes('같은 ID'));
  assert(elements.status.textContent.includes('재시도'));
  assert.strictEqual(elements.releaseHashes.textContent, '');
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )

    def test_indeterminate_activation_preserves_hashes_and_avoids_false_failure(self):
        """Post-rename durability ambiguity needs distinct verification/retry guidance."""
        scenario = r"""
  assert(elements.status.className.includes('success'), elements.status.textContent);
  const normalFetch = globalThis.fetch;
  globalThis.fetch = (url, options = {}) => {
    if (url === '/api/release' && options.method === 'POST') {
      const request = JSON.parse(options.body);
      posts.push(request);
      return Promise.resolve(jsonResponse({
        schema_version: 1,
        release_id: request.release_id,
        activated: 'indeterminate',
        manifest_sha256: 'c'.repeat(64),
        artifact_hashes: {
          'map.pgm': 'd'.repeat(64),
          'map.yaml': 'e'.repeat(64),
          'static_overlay.json': 'f'.repeat(64),
          'semantic_layer.json': '0'.repeat(64),
        },
        error: {
          code: 'activation_indeterminate',
          message: 'active pointer was replaced but directory fsync failed',
        },
      }, 201));
    }
    return normalFetch(url, options);
  };
  elements.releaseId.value = 'indeterminate-release';
  elements.activateRelease.checked = true;
  elements.release.emit('click');
  await waitFor(() => posts.length === 1, 'indeterminate activation request');
  await waitFor(() => elements.status.className.includes('error'), 'indeterminate guidance');

  assert(elements.releaseHashes.textContent.includes('release: indeterminate-release'));
  assert(elements.releaseHashes.textContent.includes('activated: indeterminate'));
  assert(elements.releaseHashes.textContent.includes('manifest: ' + 'c'.repeat(64)));
  assert(elements.status.textContent.includes('영구 반영 여부가 불명확'));
  assert(elements.status.textContent.includes('확인'));
  assert(elements.status.textContent.includes('같은 ID'));
  assert(!elements.status.textContent.includes('활성화에 실패'));
"""
        self.run_browser(
            ORIGINAL_PGM,
            hashlib.sha256(ORIGINAL_PGM).hexdigest(),
            scenario,
        )


if __name__ == "__main__":
    unittest.main()
