# 현재 코드 상세 설명

[index.html](index.html)을 브라우저에서 연다. 6개 Archify 그림과 36개 코드 근거, 26개 소스·설정 파일의 SHA-256 스냅샷을 포함한다.

- [그림 검증 및 해시](validation-summary.json)
- [코드·링크 검사](content-check.json)
- [소스 manifest](source-manifest.json)

문서만 추가했으며 운영 코드·ROS 인터페이스 변경은 없다. catkin 빌드와 rosbag/실차 신규 검증은 수행하지 않았다. 코드 변경 뒤에는 현재 파일과 manifest 해시를 비교하고 문서를 갱신해야 한다. Archify revision-pinned repository 기능 대신 Git root가 아닌 현재 workspace의 파일 스냅샷을 사용했다.
