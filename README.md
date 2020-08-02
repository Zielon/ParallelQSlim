# ParallelQSlim
Parallel mesh simplification algorithm

This work elaborates a new parallel algorithm based on quadric error metric and adaptive thresholding to simplify a triangle mesh. The approach emphasizes planar surfaces as a target to simplify. The main goal was to create a framework able to produce high quality progressive meshes for browser streaming purposes.

The project needs two libraries:
```
Installation:
    sudo apt-get install libboost-all-dev
    sudo apt install libeigen3-dev
```

Here you can read the [report](https://github.com/Zielon/QSlim/blob/master/report/report.pdf) which summarizes the whole work.

![output_1](https://github.com/Zielon/QSlim/blob/master/report/simply_9.gif)

Default reader accepts as an input mesh only those with binary encoding.

```
Usage:
     main --in ../resources/armadillo.ply --out ../output/simply.ply -f

Allowed options:
  -h [ --help ]                    Produce help message
  --in arg                         Path fine input mesh
  --out arg                        Output path of simplified mesh
  -v [ --verbose ]                 Show debug output
  -f [ --force ]                   Enable file overwrite
  -s [ --smooth ]                  Smooth the mesh using Taubin
  -w [ --weight ] arg (=0)         Quadric error weighting strategy
                                    0 = none
                                    1 = area
                                   
  -r [ --reduction ] arg (=75)      The percentage reduction which we want to 
                                   achieve; e.g. 10 of the input mesh
  -i [ --max-iter ] arg (=10)       Max iterations to perform
  -t [ --threads ] arg (=1)        Number of threads
  -q [ --quadric ] arg (=3)        Type of quadric metric
                                    3 = [geometry]
                                    6 = [geometry, color]
                                    9 = [geometry, color, normal]
                                   
  -c [ --clusters ] arg (=2)       Number of clusters e.g.
                                    2 will be 2x2x2=8, 3x3x3=27 clusters
  -m [ --attributes ] arg (=1)     Input mesh attributes
                                    1 = [geometry]
                                    2 = [geometry, color, normal]
  -a [ --aggressiveness ] arg (=3) Aggressiveness (directly relates to the 
                                   maximum permissive error) [1.0-10.0]
```