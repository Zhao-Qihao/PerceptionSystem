环境配置

Ubuntu 20.04

ROS noetic

安装opencv 4.9.0后

```bash
nano ~/.bashrc 

export OpenCV_DIR=/usr/local/opencv-4.9.0/lib/cmake/opencv4
export LD_LIBRARY_PATH=/usr/local/opencv-4.9.0/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-4.9.0/lib/pkgconfig:$PKG_CONFIG_PATH
```

cuda12.3下载

然后配置cuda环境

```bash
nano ~/.bashrc 

export CUDA_HOME=/usr/local/cuda-12.3
export PATH=${CUDA_HOME}/bin:${PATH}
export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
```



cuda11.8下载后只需sudo sh run 文件，因为TensorRT8依赖cuda11的cuBLAS .so文件





cudnn 8.9  https://developer.nvidia.com/rdp/cudnn-archive

下载https://developer.nvidia.com/downloads/compute/cudnn/secure/8.9.7/local_installers/12.x/cudnn-linux-x86_64-8.9.7.29_cuda12-archive.tar.xz/

Download cuDNN v8.2.1 (June 7, 2021)” → “cuDNN Library for Linux (x86_64)”



```bash
tar -xvf cudnn-linux-x86_64-8.9.7.29_cuda12-archive.tar.xz
cd cudnn-linux-x86_64-8.9.7.29_cuda12-archive
sudo cp include/cudnn*.h /usr/local/cuda-12.3/include/
sudo cp lib/libcudnn* /usr/local/cuda-12.3/lib64/
sudo chmod a+r /usr/local/cuda-12.3/include/cudnn*.h /usr/local/cuda-12.3/lib64/libcudnn*    
```





TensorRT8.4.3.1下载https://developer.nvidia.com/compute/machine-learning/tensorrt/secure/8.4.3/tars/tensorrt-8.4.3.1.linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

然后

```bash
cp TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz /usr/local
cd usr/local
sudo tar -xpf TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

nano ~/.bashrc 
```



再下载TensorRT 10.7.0.23

```bash
cp TensorRT-10.7.0.23 /usr/local
cd usr/local
sudo tar -xpf TensorRT-10.7.0.23.tar.gz

nano ~/.bashrc 

export TENSORRT_ROOT=/usr/local/TensorRT-10.7.0.23
export LD_LIBRARY_PATH=$TENSORRT_ROOT/lib:$LD_LIBRARY_PATH
export PATH=$TENSORRT_ROOT/bin:$PATH
```



