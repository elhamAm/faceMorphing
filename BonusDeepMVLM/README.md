## Getting Deep-MVLM

Download or clone from github

https://github.com/RasmusRPaulsen/Deep-MVLM


### Requriements

Original Developer note: The code has been tested under Windows 10 both with a GPU enabled (Titan X) computer and without a GPU (works but slow). It has been tested with the following dependencies
Our project use: Code ran with Windows 10, GPU enabled (Nvidia GeForce RXT 3070)


- Python 3.7
- Pytorch 1.2
- vtk 8.2
- libnetcdf 4.7.1 (needed by vtk)
- imageio 2.6
- matplotlib 3.1.1
- scipy 1.3.1
- scikit-image 0.15
- tensorboard 1.14
- absl-py 0.8

### Generating Landmarks

Move .obj meshes into assets, for easier relative path

use 
```
python predict.py --c configs/DTU3D-geometry.json --n yourdirectory
```

The txt files generated contain vertex positions