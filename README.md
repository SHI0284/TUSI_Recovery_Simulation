# TUSI Recovery Simulation

로켓의 발사부터 낙하산 회수까지 재생하고, **GPS·방위 센서를 이용한 PD 제어가 모터 회전·줄 당김·비행 경로에 미치는 영향**을 비교하는 시뮬레이터다.

OpenRocket 파일의 로켓 외형·저장 상승 궤적, 추진 모터의 추력 곡선, ST3235 서보 참조 사양을 사용한다. 캐노피의 공력·조향 계수와 기구 응답에는 아직 실측하지 않은 가정이 포함된다. 계산 결과는 실물 성능 검증이나 비행 펌웨어를 대신하지 않는다.

## 실행

Python 3.10 이상과 WebGL을 지원하는 브라우저가 필요하다. Python 실행에는 추가 패키지가 필요 없고, 3D 라이브러리도 저장소에 포함돼 있다.

```bash
git clone https://github.com/SHI0284/TUSI_Recovery_Simulation.git
cd TUSI_Recovery_Simulation
python3 -m simulation.server
```

[http://127.0.0.1:8765](http://127.0.0.1:8765)를 연다. 포트가 사용 중이면 `python3 -m simulation.server --port 8766`으로 실행한다. HTML 파일을 직접 열지 말고 서버를 통해 접속한다.

1. **기본 미션**으로 발사·상승·전개·회수·착지 흐름을 본다.
2. **상승 궤적** 카메라로 ORK의 횡이동과 기울기를 확인한다.
3. **수동 줄 단계 시험**에서 모터 지령·응답 그래프를 선택하고 **현재 주변 2초**로 확대한다.
4. 목표 위치·PD 이득·바람·센서를 바꾸고 다시 계산한다.
5. **회전각별 비교**와 **가정 8가지 비교**로 당김량과 조향 불확실성의 영향을 확인한다.

## 무엇을 읽으면 되나

| 목적 | 문서 |
| --- | --- |
| 시뮬레이션 흐름, 시간축, 화면·출력 사용법 | [시뮬레이션 설명](docs/simulation.md) |
| GPS에서 모터 지령까지, PD 식과 제어 코드 | [제어 설명](docs/control.md) |
| 모터·줄·캐노피·바람·센서의 물리 가정 | [물리 환경 설명](docs/physics.md) |
| 모든 설정 키, 기본값, 단위 | [파라미터 목록](docs/parameters.md) |
| 실제 파일 출처·상세 수식·보정 항목 | [기술 참고](docs/reference.md) |
| 코드 역할과 수정 위치 | [코드 구조](docs/architecture.md) |
| 발사부터 착지까지 단계별 시나리오 | [미션 시나리오](simulation/mission_scenario.md) |

## 코드 구조

```text
simulation/
  configuration.py    설정 로딩·유효 범위 검사
  model.py            회수 시뮬레이션 반복 실행
  guidance.py         센서 기반 목표 방위·PD 제어
  actuator.py         모터 지연·가감속·줄 장력·당김
  aerodynamics.py     정상 활공·비행 운동방정식
  environment.py      바람·돌풍·가상 센서
  ascent.py           ORK 상승 재생 / ENG 수직 상승
  mission.py          발사부터 착지까지 시간축 구성
  attitude.py         ORK 자세 좌표 변환
  inputs.py           ORK·ENG 입력 읽기
  geometry.py         ORK 외형 치수 추출
  sensitivity.py      가정 8가지 비교
  server.py           로컬 HTTP API
  run.py              명령줄 실행·CSV/JSON 출력
  web/                3D 화면·그래프·설정 UI
  data/               입력 자료·참조 사양·원본 해시
  tests/              물리 관계·센서·제어·미션 검사
```

## 화면 없이 계산

```bash
python3 -m simulation.run --config examples/manual_step.json
python3 -m simulation.run --config examples/gust.json --output simulation/results/gust
python3 -m simulation.run --initialization measured
python3 -m unittest discover -s simulation/tests -v
```

출력은 기본적으로 `simulation/results/`에 생성되며 Git에서 제외된다. 단위는 m·s·N·N·m, 내부 각도는 rad, 화면과 `_deg` 출력은 °다.

## 결과를 해석할 때

- ORK 상승은 **저장된 계산 결과의 재생**이다. 화면의 바람 설정을 바꿔 상승을 새로 해석하지 않는다.
- 회로도상 서보 공급은 6.5 V, 확보한 토크·속도 자료는 12 V다. 전압에 따른 성능을 임의로 환산하지 않는다.
- 두 줄 카이트의 자유 비행 안정성과 차등 당김에 따른 조향은 아직 미검증이다. 화면의 캐노피 변형은 설명용이다.
- 전개 충격·진자 운동·천의 변형·접힘·실속·원시 IMU 융합·전원 강하는 해석하지 않는다.
- 테스트는 구현과 수학 관계를 확인한다. 실제 장치의 정확도를 보증하지 않는다.

원본 입력과 제3자 자료의 출처·권리 범위는 [자료 및 라이브러리 안내](THIRD_PARTY_NOTICES.md)에 정리했다.
