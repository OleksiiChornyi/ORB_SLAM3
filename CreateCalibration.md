# Create SLAM calibration

1. We need to fill `fx`, `fy`, `cx`, `cy` for left and right camera. This values we get from `camera_matrix` field.

```
[ fx   0  cx ]
[  0  fy  cy ]
[  0   0   1 ]
```
```
camera_matrix:
  rows: 3
  cols: 3
  data: [440.87759,   0.     , 198.9307 ,
           0.     , 437.59505, 112.85471,
           0.     ,   0.     ,   1.     ]
```
```
Camera1.fx: 440.87759
Camera1.fy: 437.59505
Camera1.cx: 198.9307
Camera1.cy: 112.85471
```

2. We need to fill `k1`, `k2`, `p1`, `p2` for left and right camera. This values we get from `distortion_coefficients` field

```
[k1, k2, p1, p2, k3]
```
```
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.047731, 0.067837, 0.004981, -0.005443, 0.000000]
```
```
Camera1.k1: -0.047731
Camera1.k2: 0.067837
Camera1.p1: 0.004981
Camera1.p2: -0.005443
```

3. We need to fill `T_c1_c2` field. For this we have `ORB_SLAM3/visual_stabilization/calibration/calculate_stereo_t_c1_c2.py` script to calulate value. For it you need to replace values in script and run it to get results.
    - `R_c1` and `R_c2` are values from `rectification_matrix` in calibration of right and left camera.
    - `t1` is always zerows `[0.0, 0.0, 0.0]`.
    - `t2` is result of `Tx` / `fx` of `projection_matrix` in calubration file. 

```
rectification_matrix:
  rows: 3
  cols: 3
  data: [ 0.9972638 ,  0.03654628,  0.06425942,
         -0.0369175 ,  0.99930774,  0.00459865,
         -0.06404688, -0.00695837,  0.99792263]
```
```
R_c1 = np.array([
    [ 0.9972638 ,  0.03654628,  0.06425942],
    [-0.0369175 ,  0.99930774,  0.00459865],
    [-0.06404688, -0.00695837,  0.99792263]
])
R_c2 = ...

```
```
[fx, 0,  cx,  Tx ],
[0,  fy, cy,  0  ],
[0,  0,   1,  0  ]
```
```
projection_matrix:
  rows: 3
  cols: 4
  data: [490.55224,   0.     , 168.55494, -11.50094,
           0.     , 490.55224, 114.0898 ,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
```
```
-11.50094 / 490.55224 = -0.234
```
```
t1 = np.array([0.0, 0.0, 0.0])
t2 = np.array([-0.234, 0.0, 0.0])
```

4. Run script and map result to calibration file:

```
python3 '/home/drones/ORB_SLAM3/visual_stabilization/calibration/calculate_stereo_t_c1_c2.py'
```
```
Relative Rotation Matrix R_diff:
[[ 0.99874757 -0.05003093 -0.00045779]
 [ 0.05002387  0.99869804 -0.0099911 ]
 [ 0.00095707  0.00995569  0.99994998]]

calculate_t_diff T_diff:
[-0.23335973 -0.00855183 -0.01503671]
```
```
[R1, R2, R3, T1]
[R3, R4, R5, T2]
[R6, R7, R8, T3]
[0,  0,  0,  1 ]
```
```
Stereo.ThDepth: 60.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.99874756, -0.05003093, -0.00045779, -0.23335973,
         0.05002387,  0.99869804, -0.00999110, -0.00855183,
         0.00095707,  0.00995569,  0.99994998, -0.01503671,
         0.0, 0.0, 0.0, 1.0]

```
