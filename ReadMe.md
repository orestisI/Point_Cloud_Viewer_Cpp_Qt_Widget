# Point Cloud Viewer (c++,Qt QWidget)

![cube](./Images/cube.png)

## Description
A point cloud viewer widget (Qt) written in c++.

## Usage
```c++
...
int bufferSize = 10000; 		//Point cloud buffer size
bool enableStreaming = true;	//Enable real time streaming
pointCloudViewer_CppQtW = PointCloudViewer_CppQtW(bufferSize,enableStreaming);
...

```

## API
```c++

//Rotates the point cloud along x-axis 
void RotateX(float dPhi);

//Rotates the point cloud along y-axis 
void RotateY(float dPhi);

//Rotates the point cloud along z-axis 
void RotateZ(float dPhi);

//Enables-disables real time streaming of point
void EnableStreaming(bool enable);

//Resizes point cloud buffer size
void ResizePointCloud(int pointCloudPointsNum);

//Clears the point cloud buffer
void Refresh();

//Stream in points (old points are been replaced by new)
void StreamIn(Eigen::MatrixXf *stream);

```

