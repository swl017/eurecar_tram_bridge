## 1. 데이터 흐름
topic: /vehicle/local_wpt/frenet/origin (Type: nav_msgs/Path)
node: /eurecar_tram_bridge
topic: /vehicle/local_wpt/splined/origin/HOLDLOCALPATH (Type: nav_msgs/Path) *remapped in launch
node: /Conventional_Controller

## 2. 실차 실험 결과(Hold Local Path fieldtest 2020-10-05 월)
### 2-1 Labview psoe(/vehicle/pose) 이용시
base_link(t0) 기준 base_link(t)의 y 좌표, yaw 반전된 듯한 현상 발생.
### 2-2 ros pose(/Odometry/ekf_estimated) 이용시
잘 작동함.
### 2-3 Labview pose와 ros pose 의 x, y 좌표 반전돼있음을 확인
Labview 내부에서 작용하는 것이 있을 것으로 예상. Labview pose를 활용하는 것은 어려울 것으로 보임.
* 그런데 base_link(t0)에서 waypoint는 잘 나오는 것은 의문...
### 2-4 현재 최선의 방법
Hold Local Path 알고리즘은 위치 변화만 일관적이면 작동하므로(다른 알고리즘과 좌표 호환이 필요 없으므로) ros pose 사용할 계획.
### 2-5 (+@) rviz 상 held path 에 latency 관찰됨.
최선의 성능이 필요할 경우 Planner 알고리즘에 통합 할 필요가 있음.
