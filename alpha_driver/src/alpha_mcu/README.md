# Alpha AUV MCU

## Building

```bash
mkdir -p ~/soslab_ws
cd ~/soslab_ws
git clone https://github.com/GSO-soslab/alpha_pico
cd alpha_pico
mkdir build
cd build
PICO_PICO_SDK_FETCH_FROM_GIT=1 cmake ..
make -j4
```