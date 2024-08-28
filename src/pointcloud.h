#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>

class PoinCloudWidget : public QVTKOpenGLNativeWidget {
public:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PoinCloudWidget(QWidget* parent) : QVTKOpenGLNativeWidget(parent) 
  {
    auto renderer     = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset( new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
  }
	
};


#endif