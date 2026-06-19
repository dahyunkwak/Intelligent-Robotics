# 캠퍼스 배달 로봇 강화학습 시스템

## 팀 정보

| 항목 | 내용 |
|------|------|
| 팀명 | 코코넛 |
| 팀원 | 곽다현 |

---

## 프로젝트 설명

ROS2 + Nav2 + Gazebo 환경에서 TurtleBot3 Burger 로봇이 캠퍼스 내 여러 건물에 물품을 배달하는 강화학습 시스템

MaskablePPO 알고리즘을 사용해 배달 순서 결정, 배터리 관리, 충전 전략을 스스로 학습하며, 규칙 기반(Rule-Based) 에이전트와 성능을 비교

### 주요 기능
- 캠퍼스 맵 기반 자율 배달 (최대 6개 배달지)
- 정문에서 캠퍼스 내부 건물 두개를 랜덤으로 입력받아 배달 수행
- 배터리 상태에 따른 자율 충전 전략 학습
- Action Masking으로 불가능한 행동 제거
- AMCL 대신 Gazebo ground-truth 위치 사용 (`gt_pose_publisher` 노드 직접 구현)
- 3가지 환경 버전 비교 (v1/v2/v3)

### 환경 버전
| 버전 | 배달지 | 충전소 위치 | 학습 스텝 |
|------|--------|-------------|-----------|
| V1 | 4개 (ECC, 대강당, 학관, 중앙도서관) | 캠퍼스 중앙 (0, 0) | ~7,271 |
| V2 | 6개 (+연구협력관, 조형예술관) | 캠퍼스 중앙 (0, 0) | ~9,474 |
| V3 | 4개 (V1과 동일) | 맵 외딴곳 (16, -20) | 진행 중 |

### 평가 결과 (각 10에피소드)
| 지표 | V1 RL | V1 Rule | V2 RL | V2 Rule | V3 RL | V3 Rule |
|------|-------|---------|-------|---------|-------|---------|
| 평균 배달 수 | **15.8** | 11.0 | **16.2** | 15.3 | **15.6** | 14.8 |
| 최대 배달 수 | 16 | 17 | 17 | 17 | 16 | 16 |
| 최소 배달 수 | **14** | 5 | **16** | 14 | 14 | 14 |
| 배터리 방전 | **0회** | 1회 | **0회** | 0회 | **0회** | 0회 |
| 불필요한 충전 | 31회 | 0회 | 25회 | 0회 | 24회 | 0회 |

---

## 기술 스택

- **ROS2 Humble** - 미들웨어
- **Gazebo** - 시뮬레이터
- **Nav2** - 자율주행 (경로 계획 및 이동 제어)
- **TurtleBot3 Burger** - 로봇 플랫폼
- **Gymnasium** - RL 환경 인터페이스
- **Stable Baselines3 + sb3_contrib** - MaskablePPO 구현
- **WSL2 (Ubuntu 22.04)** - 개발 환경

---

## AI 사용 여부 및 사용 내용

본 프로젝트에서 AI 도구(Claude)를 다음 용도로 활용하였습니다.

- **코드 작성 보조**: `campus_env.py`, `gt_pose_publisher.py`, `train_ppo.py` 등 코드 디버깅 및 ROS2/Nav2 관련 오류 해결 보조
- 발표 자료 초안 작성
- **ROS2/Nav2 설정 튜닝**: WSL2 환경에서의 타임아웃, goal tolerance 등 파라미터 조정
- **발표 자료 초안 제작**: PPT 디자인 제작

AI가 생성한 코드를 직접 검토하고 수정하며 프로젝트에 적용하였으며, 실제 학습 실행 및 결과 분석은 직접 수행하였습니다.

---

## 참고 자료

### 논문
- Schulman, J. et al. (2017). *Proximal Policy Optimization Algorithms*. arXiv:1707.06347
- Hill, A. et al. (2018). *Stable Baselines*. GitHub

### GitHub
- [Stable Baselines3](https://github.com/DLR-RM/stable-baselines3)
- [sb3-contrib (MaskablePPO)](https://github.com/Stable-Baselines-Team/stable-baselines3-contrib)
- [Nav2 (Navigation2)](https://github.com/ros-planning/navigation2)
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)

### 공식 문서
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)

---

## 데모 영상

- **발표용 (배속)**: https://www.youtube.com/watch?v=G489RXDn_80
- **원본 영상 (V1)**:https://www.youtube.com/watch?v=R-MX-UMStyE
- **원본 영상 (V2)**:https://www.youtube.com/watch?v=DhYFsUyQcTk&si=iAEjJYdm5g3PQCa-
- **원본 영상 (V3)**: https://www.youtube.com/watch?v=85ZynsHOxpA&si=1jrpD2FzRyUWfYik

---

## 계획과 달라진 점
- 계획할 때는 배터리 상태를 RVIZ로 시각화하려고 하였으나 제 프로젝트 주제에서 시각화가 크게 중요하지 않다고 판단되었고 시각화 과정을 추가했을 때 강화학습이 원활이 수행되지 않아 터미널을 통해 배터리 상태를 확인 하는 것으로 교체하였습니다. 
- 보행자 인식 역시 처음에는 ground-truth 위치를 직접 받아오는 방식을 고려했으나, 실제 로봇 환경과의 일관성을 위해 LiDAR 기반 Nav2의 obstacle_layer를 활용해 보행자를 감지하고 회피하는 방식으로 변경하였습니다.
