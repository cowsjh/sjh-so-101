cmake_minimum_required(VERSION 3.8)
project(so101_description)

find_package(ament_cmake REQUIRED)

# TODO(함께): 여기에 리소스 설치 규칙을 넣는다.
#   왜? description 패키지는 컴파일할 코드가 없고, urdf/meshes/launch/rviz 폴더를
#   share/so101_description/ 아래로 "설치"해야 package:// 경로가 실제로 풀린다.

install(
    DIRECTORY
    urdf
    meshes
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
