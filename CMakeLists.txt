cmake_minimum_required(VERSION 3.8)
project(drone_mocap)

find_package(ament_cmake REQUIRED)

# 用 Python 而不是 C++：這支要在樹莓派上跑，而樹莓派編譯 C++ 一直是這個專案的痛點。
# 工作量本身很小（每秒一百次的四元數乘法），Python 綽綽有餘；
# 真的量到延遲抖動有影響時再移植（節點每 5 秒回報的頻率就是判斷依據）。
install(PROGRAMS
  scripts/mocap_to_px4_node.py
  scripts/square_flight.py
  test/fake_mocap_publisher.py
  test/hover_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
