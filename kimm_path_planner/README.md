# kimm_path_planning

## Dependency 
- [OMPL 1.5](https://github.com/ompl/ompl/tree/1.5.0) - included as submodule, needs to be installed first
- [nlohmann/json](https://github.com/nlohmann/json) 

## How to install
- Install chomp in third_pary/chomp folder (cd third_party chomp && mkdir build && cd build && sudo make install)
- Install steering_functions in third_pary/chomp folder (cd third_party steering_functions &&  mkdir build && cd build && sudo make install)
- Install mzcommon in third_pary/chomp folder (cd third_party mzcommon && mkdir build && cd build && sudo make install)
- Install main folder (mkdir build && cd build && sudo make install)

## Demo
- 모션 플래너
```
cd bin && ./Environment
```
