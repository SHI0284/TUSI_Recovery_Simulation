# 자료 및 라이브러리 안내

## Three.js

`simulation/web/vendor/`에는 Three.js r180 모듈과 OrbitControls가 포함돼 있다. MIT 라이선스이며 저작권·허가문은 [THREE-LICENSE.txt](simulation/web/vendor/THREE-LICENSE.txt)에 보존했다. 이 폴더의 원본 헤더와 라이선스 표기는 유지한다.

## 입력 파일과 설계 자료

`simulation/data/`의 ORK·ENG·CAD·회로도는 프로젝트에서 제공한 입력이다. 원본 파일의 SHA-256과 식별 정보는 `manifest.json`, `hardware/sources.json`에 기록한다. 개인 PC의 절대 경로는 배포 메타데이터에서 제거했다. 제공 자료에 별도의 오픈소스 라이선스를 부여한 것은 아니다.

## 제품 사진과 사양

캐노피 참고 사진의 출처와 해시는 `simulation/data/parafoil_visual_reference.json`에 있다. 제품 사진은 형상 비교용 외부 자료이며 MIT 라이선스 대상으로 간주하지 않는다. ST3235 자료의 제조사 출처와 참조 전압은 `simulation/data/st3235.json`에 있다.

## 연구·계산 참고

OpenRocket의 저장 자세 정의와 파라포일 조향 연구의 링크는 [기술 참고](docs/reference.md)에 보존했다. 이 저장소는 OpenRocket 실행 파일을 포함하지 않으며 제공된 ORK 저장 결과를 읽는다. 참고 연구가 이 프로젝트의 실제 캐노피 성능을 검증한 것은 아니다.
