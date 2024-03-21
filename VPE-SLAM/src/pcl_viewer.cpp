#include "utils/pcl_viewer.h"
unsigned int text_id = 0;
std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> cube;
/*
// 可视化的时候，给点云增加颜色
template <typename PointT>
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud1,
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud2,
        const std::string &id_view1,
        const std::string &id_view2)
{
    int v1(0), v2(1);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1.0, 1, v2);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);//点云颜色设置
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> single_color1(cloud1, "intensity");   //按照z字段进行渲染
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> single_color2(cloud2, "intensity");   //按照z字段进行渲染

    viewer->addPointCloud<PointT>(cloud1, single_color1, id_view1);
    viewer->addPointCloud<PointT>(cloud2, single_color2, id_view2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view1, v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view2, v2);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}
*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud1,
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud2,
        const std::string &id_view1,
        const std::string &id_view2)

{
    int v1(0), v2(1);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1.0, 1, v2);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);//点云颜色设置
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color1(cloud1, "curvature");   //按照z字段进行渲染
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color2(cloud2, "curvature");   //按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZINormal>(cloud1, single_color1, id_view1, v1);
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud2, single_color2, id_view2, v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view1, v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view2, v2);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVisCompare(
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud1,
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud2,
        const std::string &id_view1,
        const std::string &id_view2,
        const std::string &title1,
        const std::string &title2)
{
    int v1(0), v2(1);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1.0, 1, v2);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);//点云颜色设置
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color1(cloud1, "intensity");   //按照z字段进行渲染
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color2(cloud2, "intensity");   //按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZINormal>(cloud1, single_color1, id_view1, v1);
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud2, single_color2, id_view2, v2);

    viewer->addText(title1, 0, 0, 16, 1, 1, 1, "title1", v1);
    viewer->addText(title2, 0, 0, 16, 1, 1, 1, "title2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view1, v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id_view2, v2);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}
// 显示简单点云
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

// 可视化的时候，给点云增加颜色
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);//点云颜色设置
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> single_color(cloud, "intensity");   //按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZINormal>(cloud, single_color, "zy_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "zy_cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
        pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> sing_cloud(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud, sing_cloud, "zy_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "zy_cloud");
    viewer->addPointCloudNormals<pcl::PointXYZINormal, pcl::Normal>(cloud, normals, 1, 0.1, "normals");//显示法向量
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    //------------------------------------
    //-----Add shapes at cloud points-----
    //------------------------------------
    viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
                                      cloud->points[cloud->size() - 1], "line");
    viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

    //---------------------------------------
    //-----Add shapes at other locations-----
    //---------------------------------------
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(1.0);
    coeffs.values.push_back(0.0);
    viewer->addPlane(coeffs, "plane");
    coeffs.values.clear();
    coeffs.values.push_back(0.3);
    coeffs.values.push_back(0.3);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(1.0);
    coeffs.values.push_back(0.0);
    coeffs.values.push_back(5.0);
    viewer->addCone(coeffs, "cone");

    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
        pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem(1.0);

    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

    return (viewer);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getKeySym() == "r" && event.keyDown())
    {
        std::cout << "r was pressed => removing all text" << std::endl;

        char str[512];
        for (unsigned int i = 0; i < text_id; ++i)
        {
            sprintf(str, "text#%03d", i);
            viewer->removeShape(str);
        }
        text_id = 0;
    }
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
                        void *viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

        char str[512];
        sprintf(str, "text#%03d", text_id++);
        viewer->addText("clicked here", event.getX(), event.getY(), str);
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis() {
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) viewer.get());
    viewer->registerMouseCallback(mouseEventOccurred, (void *) viewer.get());

    return (viewer);
}
