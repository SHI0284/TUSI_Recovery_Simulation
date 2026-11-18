# 코드 읽는 순서와 수정 위치

먼저 `server.py → model.py → guidance.py → actuator.py → aerodynamics.py` 순서로 읽으면 한 번의 계산 흐름을 따라갈 수 있다. 화면은 계산 결과를 재생하며 물리를 별도로 계산하지 않는다.

| 변경 목적 | 파일·진입점 |
| --- | --- |
| 기본값·입력 허용 범위 | `config.json`, `configuration.validate()` |
| 회수 반복 계산·기록 항목 | `model.simulate()` |
| 목표 방위·PD·지령 제한 | `guidance.TargetPD.update()` |
| 당김량→각도 역변환 | `guidance.TargetPD.brake_to_angle()` |
| 실제 모터·줄 반응 | `actuator.Actuator.step()` / `observe()` |
| 초기 활공과 운동방정식 | `aerodynamics.trim()` / `flight_derivative()` |
| 바람·센서 오류 모델 | `environment.WindField` / `Sensors` |
| ORK/ENG 읽기 | `inputs.read_ork()` / `read_eng()` |
| 상승 모델 | `ascent.ascent()` |
| 전체 미션 전환과 CSV | `mission.build_mission()` / `mission_rows()` |
| ORK 자세·치수 | `attitude.py`, `geometry.py` |
| 가정 비교 조건 | `sensitivity.compare_assumptions()` |
| API·정적 파일 경로 | `server.Handler` |
| 화면 입력·실행·재생 이벤트 | `web/app.js` |
| 입력 폼 항목 | `web/ui/config-fields.js` |
| 2D 경로·모터·시간 그래프 | `web/ui/charts.js` |
| 브라우저의 전체 미션 시간축 | `web/ui/mission-timeline.js` |
| 3D 형상·기체 자세·카메라 | `web/scene.js` |

## 회수 반복 한 단계

1. 현재 참 상태로 센서 패킷을 생성하고 도착한 패킷만 관측으로 전달한다.
2. 제어 시각이면 PD를 갱신한다. 그 사이에는 마지막 지령을 유지한다.
3. 좌우 서보가 지령 지연·가속·하중 제한을 적용한다.
4. 실제 당김으로 CL/CD와 선회율을 갱신한다.
5. 바람을 포함한 질점 운동을 RK4로 적분한다.
6. 지면 교차를 보간하고 기체 방위·센서 나이·결과 행을 기록한다.

## 의존 방향

`guidance.py`는 `model.py`를 import하지 않는다. 따라서 제어기에 물리 모델의 참 값이 직접 새어 들어가지 않는다. `actuator.py`와 `aerodynamics.py` 역시 UI를 모른다. `configuration.py`는 입력을 검증하고 `server.py`와 `run.py`는 같은 시뮬레이션 함수를 사용한다.

기존 사용자 코드와 테스트를 위해 `model`에서 `Actuator`, `validate`, `trim`, `DEFAULTS` 등의 import를 유지한다. 실제 구현 위치는 위 표를 기준으로 찾는다.

## API

- `GET /api/config`: 기본 설정과 참조 자료.
- `POST /api/simulate`: 부분 설정 JSON을 받아 조향·무조향 결과와 미션을 반환.
- `POST /api/sweep`: 우측 회전각별 30초 비교.
- `POST /api/sensitivity`: 현재 설정에서 가정 8개 비교.

POST 본문은 `application/json`이며 알 수 없는 키·비정상 숫자·허용 범위 밖 입력은 400 응답이다. 서버는 `127.0.0.1`에만 바인딩한다.

## 테스트와 형식

```bash
python3 -m unittest discover -s simulation/tests -v
python3 -m black --check simulation
npm run format:check
```

Python은 Black, 직접 작성한 JS/HTML/CSS는 Prettier 형식을 사용한다. `web/vendor/`의 제3자 코드는 변경하지 않는다. 그래프·화면 수정 시 실제 브라우저에서 모듈 로딩, 카메라, 재생, CSV 저장도 확인한다.
