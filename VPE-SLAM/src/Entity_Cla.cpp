#include "utils/Entity_Cla.h"
#include <math.h>

void Cut_bin::Cal_edge_for_dirX(vector<Eigen::VectorXf> &coffe1, vector<Eigen::VectorXf> &coffe2, double th)
{
    for (auto git: coffe1) {
        for (auto ngit: coffe2) {
            //cout << "git : " << git.transpose() << " ngit: " << ngit.transpose() << endl;

            double n1 = git[4], q1 = git[5];
            double n2 = ngit[4], q2 = ngit[5];

            double b1 = git[1], c1 = git[2];
            double b2 = ngit[1], c2 = ngit[2];
            if (n1 == 0 || q1 == 0) {
                //cout << "git less than 0.001" << endl;
                if (n1 == 0)
                {
                    q1 = 1;
                }
                else
                    n1 = 1;
                break;
            }
            if (n2 == 0 || q2 == 0) {
                //cout << "git less than 0.001" << endl;
                if (n2 == 0)
                {
                    q2 = 1;
                }
                else
                    n2 = 1;
                break;
            }

            if ((n1 / q1 - n2 / q2) == 0) {
                cout << "parallel" << endl;
                break;
            }

            double z = ((n1 / q1) * c1 - (n2 / q2) * c2 + b2 - b1) / (n1 / q1 - n2 / q2);
            double y = (n1 / q1) * (z - c1) + b1;
            double x = this->Cent;
            Eigen::Vector3d out_vec;
            out_vec << x, y, z;
            //cout << "edgePOint" << out_vec.transpose() << endl;
            pcl::PointXYZINormal Edge_point;
            Edge_point.x = x;
            Edge_point.y = y;
            Edge_point.z = z;
            Edge_point.curvature = Cube_namespace::POINT_LABLE::EDGE;
            this->RANS_EDGE_cloud->push_back(Edge_point);
        }
    }

}
void Cut_bin::Cal_edge_for_dirY(vector<Eigen::VectorXf> &coffe1, vector<Eigen::VectorXf> &coffe2, double th)
{
    for (auto git : coffe1 ) {
        for (auto ngit : coffe2){
            //cout << "git : " << git.transpose() << endl;
            //cout << " ngit: " << ngit.transpose() << endl;

            double n1 = git[3], q1 = git[5];
            double n2 = ngit[3], q2 = ngit[5];

            double b1 = git[0], c1 = git[2];
            double b2 = ngit[0], c2 = ngit[2];

            if (n1 == 0 || q1 == 0) {
                //cout << "git less than 0.001" << endl;
                if (n1 == 0)
                {
                    q1 = 1;
                }
                else
                    n1 = 1;
                break;
            }
            if (n2 == 0 || q2 == 0) {
                //cout << "git less than 0.001" << endl;
                if (n2 == 0)
                {
                    q2 = 1;
                }
                else
                    n2 = 1;
                break;
            }

            if ((n1 / q1 - n2 / q2) == 0) {
                cout << "parallel" << endl;
                break;
            }

            double z = ((n1 / q1) * c1 - (n2 / q2) * c2 + b2 - b1) / (n1 / q1 - n2 / q2);
            double x = (n1 / q1) * (z - c1) + b1;
            double y = this->Cent;
            Eigen::Vector3d out_vec;
            out_vec << x, y, z;
            //cout << "edgePOint" << out_vec.transpose() << endl;
            pcl::PointXYZINormal Edge_point;
            Edge_point.x = x;
            Edge_point.y = y;
            Edge_point.z = z;
            Edge_point.curvature = Cube_namespace::POINT_LABLE::EDGE;
            this->RANS_EDGE_cloud->push_back(Edge_point);
        }
    }
}

void Cut_bin::Cal_Edge_points(double th_edge)
{
    //cout << "cut bin ***************" << this->ground_cofficient_vector.size() * this->no_ground_cofficient_vector.size() << endl;
    if (this->dire == Cut_namespace::DireX) {
        /* ground and no_ground*/
        Cal_edge_for_dirX(this->ground_cofficient_vector, this->no_ground_cofficient_vector, th_edge);
        Cal_edge_for_dirX(this->top_cofficient_vector, this->no_ground_cofficient_vector, th_edge);
    }
    if (this->dire == Cut_namespace::DireY) {
        Cal_edge_for_dirY(this->ground_cofficient_vector, this->no_ground_cofficient_vector, th_edge);
        Cal_edge_for_dirY(this->top_cofficient_vector, this->no_ground_cofficient_vector, th_edge);
    }
}


void Cut_float::Cal_Edge_points(double th_edge, Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
       for (auto it:this->Cut_bin_dirX_vector)
       {
           it->Cal_Edge_points(th_edge);
       }

    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:this->Cut_bin_dirY_vector)
        {
            it->Cal_Edge_points(th_edge);
        }
    }
}
void Cut_float::Get_Edge_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
        for (auto it:this->Cut_bin_dirX_vector)
        {
            *point_out += *it->RANS_EDGE_cloud;
        }
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:this->Cut_bin_dirY_vector)
        {
            *point_out += *it->RANS_EDGE_cloud;
        }
    }
}

void Cut_float::Get_Edge_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire, double th)
{

    if (dire == Cut_namespace::DireX)
    {
        for (auto it:this->Cut_bin_dirX_vector)
        {
            for (auto points:it->RANS_EDGE_cloud->points)
            {
                std::vector<int> pointIdxKNNSearch(this->K_point_limit);
                std::vector<float> pointSquaredDistance(this->K_point_limit);
                double sum = 0;
                if (this->edge_tree->nearestKSearch(points, this->K_point_limit, pointIdxKNNSearch, pointSquaredDistance) > 0)
                {
                    for (auto Dist : pointSquaredDistance)
                    {
                       sum += Dist;
                    }
                    sum /= this->K_point_limit;
                   // cout << "sum: " << sum << endl;
                } else{
                    cout << "edge tree couldn't find nearst points" << endl;
                    break;
                }

               if (sum < th)
               {
                   point_out->push_back(points);
               }
            }
        }
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:this->Cut_bin_dirY_vector)
        {
            for (auto points:it->RANS_EDGE_cloud->points)
            {
                std::vector<int> pointIdxKNNSearch(this->K_point_limit);
                std::vector<float> pointSquaredDistance(this->K_point_limit);
                double sum = 0;
                if (this->edge_tree->nearestKSearch(points, this->K_point_limit, pointIdxKNNSearch, pointSquaredDistance) > 0)
                {
                    for (auto Dist : pointSquaredDistance)
                    {
                        sum += Dist;
                    }
                    sum /= this->K_point_limit;
                     //cout << "sum: " << sum << endl;
                } else{
                    cout << "edge tree couldn't find nearst points" << endl;
                    break;
                }

                if (sum < th)
                {
                    point_out->push_back(points);
                }
            }
        }
    }
}

void Cut_bin::RANSANCE_Estimate(int min_points_num )
{
    /* for direction X********/


    if (this->cloud_ground->size() > min_points_num) {
        cloud_temp->clear();
        int residual_points_size = this->cloud_ground->size();

        pcl::PointCloud<pcl::Normal> points_normal;
        pcl::Normal normal;
        for (auto points: this->cloud_ground->points) {
            normal.normal_x = points.normal_x;
            normal.normal_y = points.normal_y;
            normal.normal_z = points.normal_z;
            normal.curvature = points.intensity;
            points_normal.points.push_back(normal);
        }

        std::vector<pcl::PointIndices> clusters;
        reg.setInputCloud(this->cloud_ground);
        reg.setInputNormals(points_normal.makeShared());
        reg.extract(clusters);
        //cout << "clusters size: " << clusters.size() << endl;
        for (int ite = 0; ite < clusters.size(); ite++) {
            pcl::PointCloud<pcl::PointXYZINormal> point_in;
            pcl::copyPointCloud(*this->cloud_ground, clusters[ite].indices, point_in);
            RANSANCE_Estimate_cluster(point_in.makeShared(), this->ground_cofficient_vector);
        }

        /*cluster test use curvature presents cluster */
/*
        for (int ite=0; ite<clusters.size(); ite++)
        {
            for (auto indic : clusters[ite].indices)
            {
                this->cloud_ground->points[indic].intensity = ite +1;
            }
        }
        */

        //for (auto it : this->ground_cofficient_vector) 改不了this->ground_cofficient_vector的值，可能是const 类型的迭代器
        //for (auto it = this->ground_cofficient_vector.begin(); it < this->ground_cofficient_vector.end(); it++) //限制x自由度
        for (int i = 0; i < this->ground_cofficient_vector.size(); i++) {
            Eigen::Vector3d dir_vec;
            Eigen::Vector3d ori_vec;
            dir_vec << this->ground_cofficient_vector[i][3],
                    this->ground_cofficient_vector[i][4],
                    this->ground_cofficient_vector[i][5];
            /*for direction X or direction Y*/
            if (this->dire == Cut_namespace::DireX)
            {
                dir_vec[0] = 0;
                this->ground_cofficient_vector[i][0] = Cent;
            }
            else if (this->dire == Cut_namespace::DireY)
            {
                dir_vec[1]  = 0;
                this->ground_cofficient_vector[i][1] = Cent;
            }
            dir_vec.normalize();

            this->ground_cofficient_vector[i][3] = dir_vec[0];
            this->ground_cofficient_vector[i][4] = dir_vec[1];
            this->ground_cofficient_vector[i][5] = dir_vec[2];

            //cout << "x: set: from " << dir_vec.transpose() << endl;
            //cout << "unit " << dir_vec.norm() << endl;
        }
    }

    if (this->cloud_no_ground->size() > min_points_num) {
        cloud_temp->clear();
        int residual_points_size = this->cloud_no_ground->size();

        pcl::PointCloud<pcl::Normal> points_normal;
        pcl::Normal normal;
        for (auto points: this->cloud_no_ground->points) {
            normal.normal_x = points.normal_x;
            normal.normal_y = points.normal_y;
            normal.normal_z = points.normal_z;
            normal.curvature = points.intensity;
            points_normal.points.push_back(normal);
        }

        std::vector<pcl::PointIndices> clusters;
        reg.setInputCloud(this->cloud_no_ground);
        reg.setInputNormals(points_normal.makeShared());
        reg.extract(clusters);
        //cout << "no ground clusters size: " << clusters.size() << endl;
        for (int ite = 0; ite < clusters.size(); ite++) {
            pcl::PointCloud<pcl::PointXYZINormal> point_in;
            pcl::copyPointCloud(*this->cloud_no_ground, clusters[ite].indices, point_in);
            RANSANCE_Estimate_cluster(point_in.makeShared(), this->no_ground_cofficient_vector);
        }

        /******cluster test **************/
        /*
        for (int ite=0; ite<clusters.size(); ite++) {
            for (auto indic: clusters[ite].indices) {
                this->cloud_no_ground->points[indic].intensity = ite + 1;
            }
        }
         */
        //for (auto it : this->ground_cofficient_vector) 改不了this->ground_cofficient_vector的值，可能是const 类型的迭代器
        //for (auto it = this->ground_cofficient_vector.begin(); it < this->ground_cofficient_vector.end(); it++) //限制x自由度
        for (int i = 0; i < this->no_ground_cofficient_vector.size(); i++) {
            Eigen::Vector3d dir_vec;
            Eigen::Vector3d ori_vec;
            dir_vec << this->no_ground_cofficient_vector[i][3],
                    this->no_ground_cofficient_vector[i][4],
                    this->no_ground_cofficient_vector[i][5];

            /*for direction X or direction Y*/
            if (this->dire == Cut_namespace::DireX)
            {
                dir_vec[0] = 0;
                this->no_ground_cofficient_vector[i][0] = Cent;
            }
            else if (this->dire == Cut_namespace::DireY)
            {
                dir_vec[1]  = 0;
                this->no_ground_cofficient_vector[i][1] = Cent;
            }

            dir_vec.normalize();

            this->no_ground_cofficient_vector[i][3] = dir_vec[0];
            this->no_ground_cofficient_vector[i][4] = dir_vec[1];
            this->no_ground_cofficient_vector[i][5] = dir_vec[2];

            //cout << "x: set: from " << dir_vec.transpose() << endl;
            //cout << "unit " << dir_vec.norm() << endl;
        }
    }

    if (this->cloud_top->size() > min_points_num) {
        cloud_temp->clear();
        int residual_points_size = this->cloud_top->size();

        pcl::PointCloud<pcl::Normal> points_normal;
        pcl::Normal normal;
        for (auto points: this->cloud_top->points) {
            normal.normal_x = points.normal_x;
            normal.normal_y = points.normal_y;
            normal.normal_z = points.normal_z;
            normal.curvature = points.intensity;
            points_normal.points.push_back(normal);
        }

        std::vector<pcl::PointIndices> clusters;
        reg.setInputCloud(this->cloud_top);
        reg.setInputNormals(points_normal.makeShared());
        reg.extract(clusters);
        //cout << "clusters size: " << clusters.size() << endl;
        for (int ite = 0; ite < clusters.size(); ite++) {
            pcl::PointCloud<pcl::PointXYZINormal> point_in;
            pcl::copyPointCloud(*this->cloud_top, clusters[ite].indices, point_in);
            RANSANCE_Estimate_cluster(point_in.makeShared(), this->top_cofficient_vector);
        }
/*

        for (int ite=0; ite<clusters.size(); ite++) {
            for (auto indic: clusters[ite].indices) {
                this->cloud_top->points[indic].intensity = ite + 1;
            }
        }
        */
        //for (auto it : this->ground_cofficient_vector) 改不了this->ground_cofficient_vector的值，可能是const 类型的迭代器
        //for (auto it = this->ground_cofficient_vector.begin(); it < this->ground_cofficient_vector.end(); it++) //限制x自由度
        for (int i = 0; i < this->top_cofficient_vector.size(); i++) {
            Eigen::Vector3d dir_vec;
            Eigen::Vector3d ori_vec;
            dir_vec << this->top_cofficient_vector[i][3],
                    this->top_cofficient_vector[i][4],
                    this->top_cofficient_vector[i][5];

            /*for direction X or direction Y*/
            if (this->dire == Cut_namespace::DireX)
            {
                dir_vec[0] = 0;
                this->top_cofficient_vector[i][0] = Cent;
            }
            else if (this->dire == Cut_namespace::DireY)
            {
                dir_vec[1]  = 0;
                this->top_cofficient_vector[i][1] = Cent;
            }

            dir_vec.normalize();

            this->top_cofficient_vector[i][3] = dir_vec[0];
            this->top_cofficient_vector[i][4] = dir_vec[1];
            this->top_cofficient_vector[i][5] = dir_vec[2];

            //cout << "x: set: from " << dir_vec.transpose() << endl;
            //cout << "unit " << dir_vec.norm() << endl;
        }
    }
}

void Cut_bin::RANSANCE_Estimate(int min_points_num, int max_iterate)
{

    if (this->cloud_ground->size() > min_points_num) {
        cloud_temp->clear();
        pcl::copyPointCloud(*this->cloud_ground, *this->cloud_temp);
        int residual_points_size = this->cloud_ground->size();
        for (int ite = 0; ite < max_iterate; ite++) {
            RANSANCE_Estimate_specific(this->cloud_temp, this->ground_cofficient_vector, residual_points_size);
        }
        //cout << "size: optimized: " << this->ground_cofficient_vector.size() << endl;
    }

    if (this->cloud_no_ground->size() > min_points_num) {
        cloud_temp->clear();
        pcl::copyPointCloud(*this->cloud_no_ground, *this->cloud_temp);
        int residual_points_size = this->cloud_no_ground->size();
        for (int ite = 0; ite < max_iterate; ite++) {
            RANSANCE_Estimate_specific(this->cloud_temp, this->no_ground_cofficient_vector, residual_points_size);
        }
        //cout << "size: optimized: " << this->ground_cofficient_vector.size() << endl;
    }

    if (this->cloud_top->size() > min_points_num) {
        cloud_temp->clear();
        pcl::copyPointCloud(*this->cloud_top, *this->cloud_temp);
        int residual_points_size = this->cloud_top->size();
        for (int ite = 0; ite < max_iterate; ite++) {
            RANSANCE_Estimate_specific(this->cloud_temp, this->top_cofficient_vector, residual_points_size);
        }
        //cout << "size: optimized: " << this->ground_cofficient_vector.size() << endl;
    }

}
void Cut_bin::RANSANCE_Estimate_specific(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud,
                                         vector<Eigen::VectorXf> &cofficient,
                                         int &residual_points_size)
{
            std::vector<int> inliers;
            std::vector<int> outliers;
            Eigen::VectorXf Cofficient;
            if (residual_points_size <= this->inline_points_th) // 剩余待拟合的点数太少
            {
                return;
            }
            pcl::SampleConsensusModelLine<pcl::PointXYZINormal>::Ptr
                    model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZINormal>(in_cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac(model_l);

            ransac.setDistanceThreshold(this->Cent_range * this->RAN_Cent_range_fit_radio);
            ransac.computeModel();
            ransac.getInliers(inliers);
            ransac.getModelCoefficients(Cofficient);
            if ( inliers.size() <= this->outliers_points_th) //拟合出来的点数太少
            {
                return;
            }

            for (int i = 0; i < in_cloud->size(); i++)
            {

                bool is_in = false;
                for (auto it_in : inliers)
                {
                    if (it_in == i)
                    {
                        is_in = true;
                        break;
                    }
                }
                if (!is_in)
                {
                    outliers.push_back(i);
                }
            }
            //只留下没有拟合到的点
            pcl::copyPointCloud(*in_cloud, outliers, *in_cloud);
            residual_points_size = in_cloud->size();
            cofficient.push_back(Cofficient);
}

void Cut_bin::RANSANCE_Estimate_cluster(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud,
                                         vector<Eigen::VectorXf> &cofficient)
{
    std::vector<int> inliers;
    std::vector<int> outliers;
    Eigen::VectorXf Cofficient;
    if (in_cloud->size() <= this->inline_points_th) // 剩余待拟合的点数太少
    {
        return;
    }
    pcl::SampleConsensusModelLine<pcl::PointXYZINormal>::Ptr
            model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZINormal>(in_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac(model_l);

    ransac.setDistanceThreshold(this->Cent_range * this->RAN_Cent_range_fit_radio);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(Cofficient);
    if ( inliers.size() <= this->outliers_points_th) //拟合出来的点数太少
    {
        return;
    }

    cofficient.push_back(Cofficient);
}

void Cut_bin::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_in)
{
    clear_points();
  //  cout << "inside Cut_bin: bin num: " << point_in->points.size() << endl;
    if (dire == Cut_namespace::DireX)
    {
        for (auto it:point_in->points)
        {
            if ((it.x > Cent - Cent_range) && (it.x < Cent + Cent_range))
            {
                bin_points->push_back(it);
                switch (int(it.curvature)) {
                    case Cube_namespace::POINT_LABLE::GROUND :
                        cloud_ground->points.push_back(it);
                        break;

                    case Cube_namespace::POINT_LABLE::NO_GROUND :
                        cloud_no_ground->points.push_back(it);
                        break;

                    case Cube_namespace::POINT_LABLE::TOP :
                        cloud_top->points.push_back(it);
                        break;
                }
            }
        }

    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:point_in->points)
        {
            if ((it.y > Cent - Cent_range) && (it.y < Cent + Cent_range))
            {
                bin_points->push_back(it);
                switch (int(it.curvature)) {
                    case Cube_namespace::POINT_LABLE::GROUND :
                        cloud_ground->points.push_back(it);
                        break;

                    case Cube_namespace::POINT_LABLE::NO_GROUND :
                        cloud_no_ground->points.push_back(it);
                        break;

                    case Cube_namespace::POINT_LABLE::TOP :
                        cloud_top->points.push_back(it);
                        break;
                }
            }
        }
    }
   // cout << "inside Cut_bin: bin points size: " << bin_points->size() << endl;

}
Cut_bin::Cut_bin(double cent, double Cent_range, Cut_namespace::DIRECTORION dire):Cent(cent), Cent_range(Cent_range), dire(dire)
{
    bin_points.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    cloud_top.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    cloud_ground.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    cloud_no_ground.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    RANS_EDGE_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());

    reg.setMinClusterSize(this->reg_min_size);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    reg.setNumberOfNeighbours(this->regSearchKnumber);
    reg.setSmoothnessThreshold(this->regSmoothnessTh);
    reg.setCurvatureThreshold(this->regCurvatureTh);

}

void Cut_bin::clear_points(void)
{
    bin_points->clear();
    cloud_top->clear();
    cloud_ground->clear();
    cloud_no_ground->clear();
    RANS_EDGE_cloud->clear();
    cloud_temp->clear();
    vector<Eigen::VectorXf>().swap(this->ground_cofficient_vector);
    vector<Eigen::VectorXf>().swap(this->no_ground_cofficient_vector);
    vector<Eigen::VectorXf>().swap(this->top_cofficient_vector);
    //cout << "Cut bin: clear points: " << this->ground_cofficient_vector.size() << endl;
}
double Cut_float::Centrange_Inc(double Cent_range, int id, int segment_num)
{
    //cout << "range inc: " <<Cent_range*(1 + id / segment_num) << endl;
       return Cent_range*(1 + id / segment_num);
}

void Cut_float::get_bin_clouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
        for (auto it:this->Cut_bin_dirX_vector) {
            if (it->bin_points->size() > 0) {
                *point_out += *it->bin_points;
            }
        }
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:this->Cut_bin_dirY_vector) {
            if (it->bin_points->size() > 0) {
                *point_out += *it->bin_points;
            }
        }
    }
}
void Cut_float::get_bin_clouds_specific(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
        for (auto it:this->Cut_bin_dirX_vector) {
            if (it->cloud_ground->size() > 0) {
                *point_out += *it->cloud_ground;
            }
            if (it->cloud_no_ground->size() > 0) {
                *point_out += *it->cloud_no_ground;
            }
            if (it->cloud_top->size() > 0) {
                *point_out += *it->cloud_top;
            }
        }
/*
       for (auto it: point_out->points)
       {
          cout << " curvature: " << it.curvature;
       }
       */
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it:this->Cut_bin_dirY_vector) {
            if (it->cloud_ground->size() > 0) {
                *point_out += *it->cloud_ground;
            }
            if (it->cloud_no_ground->size() > 0) {
                *point_out += *it->cloud_no_ground;
            }
            if (it->cloud_top->size() > 0) {
                *point_out += *it->cloud_top;
            }
        }
    }
}

void Cut_float::get_bin_clouds(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_out, Cut_namespace::DIRECTORION dire, int specific_id)
{
    if (dire == Cut_namespace::DireX)
    {
       *point_out = *this->Cut_bin_dirX_vector[specific_id]->bin_points;
    }
    else if (dire == Cut_namespace::DireY)
    {
        *point_out = *this->Cut_bin_dirY_vector[specific_id]->bin_points;
    }
}

void Cut_float::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_in)
{
   for (auto it : this->Cut_bin_dirX_vector)
   {
       it->Put_points(point_in);
   }
    for (auto it : this->Cut_bin_dirY_vector)
    {
        it->Put_points(point_in);
    }
    /*for edge points limit*/
    //edge_tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());
    edge_tree->setInputCloud(point_in);
}

Cut_float::Cut_float(double Segment_interval, double Cent_range, double Cut_bin_dirX_num, double Cut_bin_dirY_num)
:Segment_interval(Segment_interval), Cent_range(Cent_range),
Cut_bin_dirX_num(Cut_bin_dirX_num), Cut_bin_dirY_num(Cut_bin_dirY_num)
{
    for (int i = 0; i < Cut_bin_dirX_num; i++)
    {
        Cut_bin_dirX_vector.push_back(new Cut_bin(Segment_interval*double(i), Centrange_Inc(Cent_range, i, 25), Cut_namespace::DireX));
    }
    for (int i = -(Cut_bin_dirY_num / 2); i < Cut_bin_dirY_num / 2; i++)
    {
        Cut_bin_dirY_vector.push_back(new Cut_bin(Segment_interval*i, Cent_range, Cut_namespace::DireY));
    }

     edge_tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());

}
void Cut_float::RANSANCE_Estimate(int id, Cut_namespace::DIRECTORION dire,
                                  int min_points_num, int max_iterate)
{
    if (dire == Cut_namespace::DireX)
    {
        this->Cut_bin_dirX_vector[id]->RANSANCE_Estimate(min_points_num, max_iterate);
    }
    else if (dire == Cut_namespace::DireY)
    {
        this->Cut_bin_dirY_vector[id]->RANSANCE_Estimate(min_points_num, max_iterate);
    }
}

void Cut_float::RANSANCE_Estimate(int min_points_num, int max_iterate, Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
        for (auto it : this->Cut_bin_dirX_vector)
        it->RANSANCE_Estimate(min_points_num, max_iterate);
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it : this->Cut_bin_dirY_vector)
            it->RANSANCE_Estimate(min_points_num, max_iterate);
    }
}

void Cut_float::RANSANCE_Estimate(int min_points_num,  Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    {
        for (auto it : this->Cut_bin_dirX_vector)
            it->RANSANCE_Estimate(min_points_num );
    }
    else if (dire == Cut_namespace::DireY)
    {
        for (auto it : this->Cut_bin_dirY_vector)
            it->RANSANCE_Estimate(min_points_num );
    }
}


void Cube_float_super::clear_occupyed_cross_bin(void)
{
}

Cube_float_super::Cube_float_super(double leng_th, int bin_num, int  cross_bin_num ):Cube_float(leng_th, bin_num)
{
    this->cross_bin_num = cross_bin_num;
    Cube_namespace::cross_bin cross_bin;
    for (int i = 0; i < cross_bin_num; i++)
    {
        cross_bin.used = false;
        this->cross_bin_vector.push_back(cross_bin);
    }
}
/*step_inc: 箱体每次迭代向y轴移动的距离
 * min_low_num: 最小可满足条件的点数*/
void Cube_float_super::cube_fit(double step_inc, int min_low_num)
{
    Size_evau_x(); //填充cross_bin_vector中的cube_float
    int step_size = Cube_float::Sx / step_inc; //移动次数

   for (int i = 0; i < cross_bin_vector.size(); i++)
   {
       cout << "present cube step: " << i << " cube_fit cross_bin_vector size: " << cross_bin_vector.size() << endl;
       int max_bin_num = 0;
       if(cross_bin_vector[i].used == true)
       {
           cout << "*******************************************************************cross_bin used size: " << i << endl;
            for (int j = 0; j < step_size; j++)
           {
                cout << "^^^^^^^^^^^^^^^^^^^^^^^" << endl;
                cout <<"step_size: " << step_size << " present step: " << j << endl;
               Cube_float *cube_float_temp = new Cube_float(*cross_bin_vector[i].cube_float);
               cout << "add: " << cube_float_temp->intesertion_plan_point << " " << cross_bin_vector[i].cube_float->intesertion_plan_point << endl;
                cout << "made_by_plan: " << step_inc*j << endl;
               //cube_float_temp.cube_made_by_plan(step_inc*j); //根据intesection_points创建箱体
               cube_float_temp->cube_made_by_plan(step_inc*j); //根据intesection_points创建箱体
               cout << "put points!" << endl;
               cube_float_temp->put_points_in_occupyed_cube(); //放入points_in到箱体中
               cout << "cal_compont!" << endl;
               cube_float_temp->cal_compont();//计算元素比率
               cout << "clear_occupyed_bin_low_num!" << endl;
               cube_float_temp->clear_occupyed_bin_low_num(min_low_num);//清除元素点比率最低的箱子
               int occp_bin_num =cube_float_temp->get_bin_num_occupyed();
               cout << "step: " << j << "  ==============>Cube_float bin num: " << occp_bin_num << endl;
               if ( occp_bin_num > max_bin_num) //获取有最多箱体的cross_bin
               {
                    delete cross_bin_vector[i].cube_float;
                    cross_bin_vector[i].cube_float = cube_float_temp;

                   for (auto cross  = cube_float_temp->cube_float_bin_vec.begin();
                        cross <     cube_float_temp->cube_float_bin_vec.end(); cross++)
                   {
                       cout << "cube_message ininiinside:  " <<  cross->Cent_x << " " << cross->Cent_y << " " << cross->Cent_z << endl;
                   }

                    max_bin_num = occp_bin_num;
                    cout << "???????MAX bin cnt : " << max_bin_num << endl;
               }
           }

           for (auto cross  = cross_bin_vector[i].cube_float->cube_float_bin_vec.begin();
                cross <     cross_bin_vector[i].cube_float->cube_float_bin_vec.end(); cross++)
           {
               cout << "cube_message inside:  " <<  cross->Cent_x << " " << cross->Cent_y << " " << cross->Cent_z << endl;
           }
       }

   }
}


void Cube_float_super::plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                                      int k_search, double radio_th)
{
    /*只算出了初始情况箱体的位置*/
    Cube_float::plan_intersection_points_cal_bysuper(no_ground_points, target, k_search, radio_th);

    /*把intersection_points放入箱体中*/
    Cube_float::put_intersection_points_in_occupyed_cube();
}

/*以横向_x轴分类箱体并不断的向新的横向箱体加入intersection_points*/
int Cube_float_super::Size_evau_x(void)
{
    cout << "Size_evau_x::next_empty_id: " << Cube_float::next_empty_id << endl;
    cout << "Cube_float: points size all: " << Cube_float::get_pointsNum_of_occupyed_bin() << endl;
    for (int i = 0; i < Cube_float::next_empty_id; i++)
    {
        //cout << "step: " << i << endl;
        int id = 0;
        if (Cube_float::cube_float_bin_vec[i].used == true)
        {
            id = (Cube_float::cube_float_bin_vec[i].Cent_x - Cube_float::Sx / 2) / Sx;
            //cout << "id: " << id << endl;
            if (cross_bin_vector[id].used == false)
            {
                cross_bin_vector[id].used = true;
                /*这里每帧数据进来时都会新申请一个Cube_float类，没有做指针指向地址的内存消除，可能存在内存累计的情况*/
                cross_bin_vector[id].cube_float =  new Cube_float(Cube_float::Sx, int(Cube_float::bin_num / 5));
                /*这里放入的是当前帧的intersection_points*/
                cross_bin_vector[id].cube_float->Put_points(Cube_float::points_in);
               // cout << "cross_give inpoints " << endl;
                *cross_bin_vector[id].cube_float->intesertion_plan_point += *Cube_float::cube_float_bin_vec[i].points;
            } else
            {
                //cout << "cross_give inters " << endl;
                *cross_bin_vector[id].cube_float->intesertion_plan_point += *Cube_float::cube_float_bin_vec[i].points;
            }
        }
    }

    cout << "cross_bin_vector all points num size: " << get_pointsNum_of_occupyed_bin() << endl;
    /*
    for (int i = 0; i < cross_bin_vector.size(); i++)
    {
        if (cross_bin_vector[i].used == true)
        {
            cout << "cross_bin_vector: " << i << "  points_in size: " << cross_bin_vector[i].cube_float->points_in->size() << endl;
            cout << "intesertion_plan_point size: " << cross_bin_vector[i].cube_float->intesertion_plan_point->size() << endl;
        }
    }
     */
    cout << "evau_x: end? " << endl;
}

int Cube_float_super::get_pointsNum_of_occupyed_bin(int cnt_input)
{
    int interPoints_num = 0;
    int cnt = 0;
    for (std::vector<Cube_namespace::cross_bin>::iterator it = cross_bin_vector.begin(); it < cross_bin_vector.end(); it++)
    {
      if (it->used == true)
      {
          cout << "id:  " << cnt++ << " intesertion size: " << it->cube_float->intesertion_plan_point->points.size() << endl;
          interPoints_num += it->cube_float->intesertion_plan_point->points.size();
          if (cnt > cnt_input)
          {
              break;
          }
      }
    }
    return interPoints_num;
}

void Cube_float_super::get_intesPoints_cross_bin_vector_by_different_caverature(pcl::PointCloud<pcl::PointXYZINormal>::Ptr draw_points ,int cnt_input)
{
int interPoints_num = 0;
int cnt = 0;
pcl::PointXYZINormal po_temp;
    for (std::vector<Cube_namespace::cross_bin>::iterator it = cross_bin_vector.begin(); it < cross_bin_vector.end(); it++)
    {
        cnt++;
        //cout << "vector " << cnt++ << endl;
        if (it->used == true)
        {
          // cout << "get intesPoints cross bin cnt: " << cnt<< endl;
           //cout << "intesPoints size: " << it->cube_float->intesertion_plan_point->points.size() << endl;
           int cnt_point = 0;
            /*for(auto point= it->cube_float->intesertion_plan_point->points.begin();
            point < it->cube_float->intesertion_plan_point->points.end(); it++)
             */
            for (int j = 0; j < it->cube_float->intesertion_plan_point->points.size(); j++)
            {
               // cout << "cnt_point: " << cnt_point++ << endl;
               cnt_point++;

                po_temp = it->cube_float->intesertion_plan_point->points[j];
                po_temp.curvature = cnt*5;

                draw_points->push_back(po_temp);
            }

        }
        if (cnt > cnt_input)
        {
            break;
        }
    }
    cout << "endl" << endl;
}

void Cube_float::viewer_add_circle(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    PointType cent;
    for (int i = 0; i < next_empty_id; i++)
    {
        cent.x = cube_float_bin_vec[i].Cent_x;
        cent.y = cube_float_bin_vec[i].Cent_y;
        cent.z = cube_float_bin_vec[i].Cent_z;
        viewer->addSphere(cent, 0.1, 0.5, 0.5, 0.0, boost::chrono::to_string(i));
    }
}

void Cut_float::Draw_line(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               int id, Cut_namespace::DIRECTORION dire)
{
    //????????如果cnt 用于连续帧处理，那么static cnt就必须修改
    static int cnt = 0;
    if (dire == Cut_namespace::DireX)
    {
        //*****************ground*****************
        for (auto it:this->Cut_bin_dirX_vector[id]->ground_cofficient_vector) {
                Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
                Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
 //               cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

                 pcl::ModelCoefficients line_coeff;
                line_coeff.values.resize (6);    // We need 6 values
                line_coeff.values[0] = it[0];
                line_coeff.values[1] = it[1];
                line_coeff.values[2] = it[2];

                line_coeff.values[3] = it[3];
                line_coeff.values[4] = it[4];
                line_coeff.values[5] = it[5];
                viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }
        //**********************no_ground****************************
        for (auto it:this->Cut_bin_dirX_vector[id]->no_ground_cofficient_vector) {
            Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
            Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
//            cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

            pcl::ModelCoefficients line_coeff;
            line_coeff.values.resize (6);    // We need 6 values
            line_coeff.values[0] = it[0];
            line_coeff.values[1] = it[1];
            line_coeff.values[2] = it[2];

            line_coeff.values[3] = it[3];
            line_coeff.values[4] = it[4];
            line_coeff.values[5] = it[5];
            viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }
        //************************top**********************************
                for (auto it:this->Cut_bin_dirX_vector[id]->top_cofficient_vector) {
                Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
                Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
             //   cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

                 pcl::ModelCoefficients line_coeff;
                line_coeff.values.resize (6);    // We need 6 values
                line_coeff.values[0] = it[0];
                line_coeff.values[1] = it[1];
                line_coeff.values[2] = it[2];

                line_coeff.values[3] = it[3];
                line_coeff.values[4] = it[4];
                line_coeff.values[5] = it[5];
                viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }

    }
    else if (dire == Cut_namespace::DireY)
    {
        //*****************ground*****************
        for (auto it:this->Cut_bin_dirY_vector[id]->ground_cofficient_vector) {
            Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
            Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
            //cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

            pcl::ModelCoefficients line_coeff;
            line_coeff.values.resize (6);    // We need 6 values
            line_coeff.values[0] = it[0];
            line_coeff.values[1] = it[1];
            line_coeff.values[2] = it[2];

            line_coeff.values[3] = it[3];
            line_coeff.values[4] = it[4];
            line_coeff.values[5] = it[5];
            viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }
        //**********************no_ground****************************
        for (auto it:this->Cut_bin_dirY_vector[id]->no_ground_cofficient_vector) {
            Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
            Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
            //cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

            pcl::ModelCoefficients line_coeff;
            line_coeff.values.resize (6);    // We need 6 values
            line_coeff.values[0] = it[0];
            line_coeff.values[1] = it[1];
            line_coeff.values[2] = it[2];

            line_coeff.values[3] = it[3];
            line_coeff.values[4] = it[4];
            line_coeff.values[5] = it[5];
            viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }
        //************************top**********************************
        for (auto it:this->Cut_bin_dirY_vector[id]->top_cofficient_vector) {
            Eigen::Vector3d dre_vec(it[3], it[4], it[5]);
            Eigen::Vector3d ori_vec(it[0], it[1], it[2]);
            //cout << "dre_vec: " << dre_vec << "  ori_vec: " << ori_vec << endl;

            pcl::ModelCoefficients line_coeff;
            line_coeff.values.resize (6);    // We need 6 values
            line_coeff.values[0] = it[0];
            line_coeff.values[1] = it[1];
            line_coeff.values[2] = it[2];

            line_coeff.values[3] = it[3];
            line_coeff.values[4] = it[4];
            line_coeff.values[5] = it[5];
            viewer->addLine(line_coeff, boost::chrono::to_string(cnt++));
        }

    }
}

void Cut_float::Draw_line(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               Cut_namespace::DIRECTORION dire)
{
    if (dire == Cut_namespace::DireX)
    for (int id = 0; id < this->Cut_bin_dirX_vector.size(); id++)
    {
        Draw_line(viewer, id, dire);
    }

    if (dire == Cut_namespace::DireY)
    for (int id = 0; id < this->Cut_bin_dirY_vector.size(); id++)
    {
        Draw_line(viewer, id, dire);
    }
}
void Cube_float_super::viewer_add_cube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    int cnt = 0;
    int cnt_cross_bin = 0;
    for (std::vector<Cube_namespace::cross_bin>::iterator it = cross_bin_vector.begin(); it < cross_bin_vector.end(); it++)
    {
        cnt_cross_bin++;
        if (it->used == true)
        {
            int i = 0;
            for ( ; i < it->cube_float->next_empty_id; i++)
            {
                cout << "viewer cube: " << i + cnt << endl;
                viewer->addCube(cube_float_bin_vec[i].begin_x,
                                cube_float_bin_vec[i].end_x,
                                cube_float_bin_vec[i].begin_y,
                                cube_float_bin_vec[i].end_y,
                                cube_float_bin_vec[i].begin_z,
                                cube_float_bin_vec[i].end_z, 0.5, 0.5, 0.0, boost::chrono::to_string(i+cnt));
                cout << "cube_message:  " << cnt_cross_bin << " " << cube_float_bin_vec[i].Cent_x << " " << cube_float_bin_vec[i].Cent_y << " " << cube_float_bin_vec[i].Cent_z << endl;
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1,boost::chrono::to_string(i + cnt));
            }
            cnt += i;
        }
    }

}

void Cube_float::viewer_add_cube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    for (int i = 0; i < next_empty_id; i++)
    {

        viewer->addCube(cube_float_bin_vec[i].begin_x,
                        cube_float_bin_vec[i].end_x,
                        cube_float_bin_vec[i].begin_y,
                        cube_float_bin_vec[i].end_y,
                        cube_float_bin_vec[i].begin_z,
                        cube_float_bin_vec[i].end_z, 0.5, 0.5, 0.0, boost::chrono::to_string(i));

        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1,boost::chrono::to_string(i));
    }
}

Cube_float Cube_float::operator=(Cube_float &cube_in)
{
    cout << "operator= " << endl;
}

Cube_float::Cube_float(Cube_float& cube_in)
{
    this->Sx = cube_in.Sx;
    this->Sy = cube_in.Sy;
    this->Sz = cube_in.Sz;
    this->bin_num = cube_in.bin_num;
    kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
    intesertion_plan_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    intesertion_edge_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    this->points_in.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    for (int i = 0; i < bin_num; i++) {
        Cube_namespace::cube_float_bin bin_temp = {0};
        bin_temp.points.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        bin_temp.id = i;
        bin_temp.Cent_x = 0;
        bin_temp.Cent_y = 0;
        bin_temp.Cent_z = 0;
        bin_temp.used = false;
        bin_temp.begin_x = 0;
        bin_temp.end_x = 0;
        bin_temp.begin_y = 0;
        bin_temp.begin_y = 0;
        bin_temp.end_z = 0;
        bin_temp.end_z = 0;
        bin_temp.compont = {0};
        cube_float_bin_vec.push_back(bin_temp);
    }
    this->Put_points(cube_in.points_in);
    *this->intesertion_plan_point = *cube_in.intesertion_plan_point;
    cout << "cube_float reconstrucion: points_in size: " <<  this->points_in->size() << " intesertion size: " << intesertion_plan_point->points.size() << endl;
}

Cube_float::Cube_float(double leng_th, int bin_num) : Sx(leng_th), Sy(leng_th), Sz(leng_th), bin_num(bin_num)
{
    kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
    intesertion_plan_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    intesertion_edge_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    this->points_in.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    for (int i = 0; i < bin_num; i++) {
        Cube_namespace::cube_float_bin bin_temp = {0};
        bin_temp.points.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        bin_temp.id = i;
        bin_temp.Cent_x = 0;
        bin_temp.Cent_y = 0;
        bin_temp.Cent_z = 0;
        bin_temp.used = false;
        bin_temp.begin_x = 0;
        bin_temp.end_x = 0;
        bin_temp.begin_y = 0;
        bin_temp.begin_y = 0;
        bin_temp.end_z = 0;
        bin_temp.end_z = 0;
        bin_temp.compont = {0};
        cube_float_bin_vec.push_back(bin_temp);
    }
}
Cube_float::Cube_float(double Sx, double Sy, double Sz, int bin_num) : Sx(Sx), Sy(Sy), Sz(Sz), bin_num(bin_num)
{
    kd_tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
    intesertion_plan_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    intesertion_edge_point.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    this->points_in.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    for (int i = 0; i < bin_num; i++) {
        Cube_namespace::cube_float_bin bin_temp = {0};
        bin_temp.points.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        bin_temp.id = i;
        bin_temp.Cent_x = 0;
        bin_temp.Cent_y = 0;
        bin_temp.Cent_z = 0;
        bin_temp.used = false;
        bin_temp.begin_x = 0;
        bin_temp.end_x = 0;
        bin_temp.begin_y = 0;
        bin_temp.begin_y = 0;
        bin_temp.end_z = 0;
        bin_temp.end_z = 0;
        bin_temp.compont = {0};
        cube_float_bin_vec.push_back(bin_temp);
    }
}
void Cube_float::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in)
{
    next_empty_id = 0;
    this->points_in->clear();
    intesertion_edge_point->clear();
    intesertion_plan_point->clear();
    *this->points_in = *points_in;

    for (std::vector<Cube_namespace::cube_float_bin>::iterator it = cube_float_bin_vec.begin();
        it < cube_float_bin_vec.end(); it++)
    {
        if (it->used == true)
        {
            it->points->points.clear();
            it->Cent_x = 0;
            it->Cent_y = 0;
            it->Cent_z = 0;
            it->used = false;
            it->begin_x = 0;
            it->end_x = 0;
            it->begin_y = 0;
            it->begin_y = 0;
            it->end_z = 0;
            it->end_z = 0;
            it->compont = {0};
        }
    }
}

void Cube_float::plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                       pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                       int k_search, double radio_th)
{
    kd_tree->setInputCloud(target);
    vector<int> point_search_idx;
    vector<float> point_search_dists;
    double sum_dis = 0, ave_dis = 0;
    //cout << "inside NO-ground_points size: " << no_ground_points->points.size() << endl;
    for (auto it : no_ground_points->points)
    {
        sum_dis = 0; ave_dis = 0;
        vector<int>().swap(point_search_idx);
        vector<float>().swap(point_search_dists);
        kd_tree->nearestKSearch(it, k_search, point_search_idx, point_search_dists);
        //cout << "1sum_dis: " << sum_dis << endl;

        if (point_search_idx.size() > 0)
        {
           // cout << "point search_idx size: " << point_search_idx.size() << endl;
            for (auto it : point_search_dists)
            {
                sum_dis += it;
            }

            ave_dis = sum_dis / double (k_search);
            //cout << "sum_dis: " << sum_dis << endl;
            if (ave_dis < radio_th)
            {
                //cout << "ave_dis: " << ave_dis << endl;
                intesertion_plan_point->push_back(it);
            }
        }

    }
    //cout << "inside: intersertion_plan_point size: " << intesertion_plan_point->size() << endl;
    cube_made_by_plan();
    put_points_in_occupyed_cube();
    cal_compont();
}

void Cube_float::plan_intersection_points_cal(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                           pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                            double radio_th)
{
    kd_tree->setInputCloud(target);
    vector<int> point_search_idx;
    vector<float> point_search_dists;
    double sum_dis = 0, ave_dis = 0;
    //cout << "inside NO-ground_points size: " << no_ground_points->points.size() << endl;
    for (auto it : no_ground_points->points)
    {
        sum_dis = 0; ave_dis = 0;
        vector<int>().swap(point_search_idx);
        vector<float>().swap(point_search_dists);
        kd_tree->radiusSearch(it, Sx, point_search_idx, point_search_dists);
        if (point_search_idx.size() > 0)
        {
            for (auto it : point_search_dists)
            {
                sum_dis += it;
            }
            ave_dis = sum_dis / double (point_search_idx.size());
            //cout << "sum_dis: " << sum_dis << endl;
            if (ave_dis < radio_th)
            {
                //cout << "ave_dis: " << ave_dis << endl;
                intesertion_plan_point->push_back(it);
            }
        }

    }
    //cout << "inside: intersertion_plan_point size: " << intesertion_plan_point->size() << endl;
    cube_made_by_plan();
    put_points_in_occupyed_cube();
    cal_compont();
}

void Cube_float::plan_intersection_points_cal_bysuper(pcl::PointCloud<pcl::PointXYZINormal>::Ptr no_ground_points,
                                           pcl::PointCloud<pcl::PointXYZINormal>::Ptr target,
                                           int k_search, double radio_th)
{
    kd_tree->setInputCloud(target);
    vector<int> point_search_idx;
    vector<float> point_search_dists;
    double sum_dis = 0, ave_dis = 0;
    //cout << "inside NO-ground_points size: " << no_ground_points->points.size() << endl;
    for (auto it : no_ground_points->points)
    {
        sum_dis = 0; ave_dis = 0;
        vector<int>().swap(point_search_idx);
        vector<float>().swap(point_search_dists);
        kd_tree->nearestKSearch(it, k_search, point_search_idx, point_search_dists);
        //cout << "1sum_dis: " << sum_dis << endl;

        if (point_search_idx.size() > 0)
        {
            // cout << "point search_idx size: " << point_search_idx.size() << endl;
            for (auto it : point_search_dists)
            {
                sum_dis += it;
            }

            ave_dis = sum_dis / double (k_search);
            //cout << "sum_dis: " << sum_dis << endl;
            if (ave_dis < radio_th)
            {
                //cout << "ave_dis: " << ave_dis << endl;
                intesertion_plan_point->push_back(it);
            }
        }
    }
    cout << "intesertion points size: " << intesertion_plan_point->size() << endl;
    cube_made_by_plan();
}

void Cube_float::put_intersection_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_points)
{
   intesertion_plan_point->points.clear();
   pcl::copyPointCloud(*in_points, *intesertion_plan_point);
}
/*
 * 根据intersertion_plan_points制作cube
 */
void Cube_float::cube_made_by_plan(double biase)
{

    int sertion_points_cnt = 0;
    int cube_used_cnt = 0;
    int cube_create_cnt = 0;


    //cout << "inside: intersertion size: "<< intesertion_plan_point->size() << endl;
    //cout << "inside: next_empty_id: " << next_empty_id << endl;
  for(auto it : intesertion_plan_point->points)
  {
      sertion_points_cnt++;
      //cout << "it: " << it << endl;
      for (std::vector<Cube_namespace::cube_float_bin>::iterator it_bin = cube_float_bin_vec.begin();
           it_bin < cube_float_bin_vec.end(); it_bin++)
      {
         if (it_bin->used == true)
         {
             //cout << "sertion_points_cnt: " << sertion_points_cnt - 1 << "  "
            if ((it.x >= it_bin->begin_x) && (it.x < it_bin->end_x))
                if ((it.y >= it_bin->begin_y) && (it.y < it_bin->end_y))
                    if ((it.z >= it_bin->begin_z) && (it.z < it_bin->end_z))
                    {
                        cube_used_cnt++;
                        //cout << "cube_used_cnt: " << cube_used_cnt << endl;
                        //it_bin->points->push_back(it);
                        break;
                    }
         }
         else
         {
             cube_create_cnt++;
             if (next_empty_id >= bin_num)
             {
                 cout << "FLOAT QUBE: WARING BIN_NUM OVERFLOING" << endl;
             }
             //cout << "Sx: " << Sx << "  Sy : " << Sy << "  Sz: " << Sz << " it: "  << it << endl;
            /*
             double cent_x  = round(it.x/Sx)*Sx + Sx / 2;
             double cent_y  = round(it.y/Sy)*Sy + Sy / 2;
             double cent_z  = round(it.z/Sz)*Sz + Sz / 2;
             */
            /*
            double a = -1.2, b = 2;
            cout << "math test " <<  a - fmod(a, b) - b /2 << endl;
             */
            double cent_x =0, cent_y = 0, cent_z = 0;

            if (it.x > 0)
             cent_x  = it.x - fmod(it.x, Sx)  + Sx / 2;
            else
                cent_x  = it.x - fmod(it.x, Sx)  - Sx / 2;

            double temp_y = it.y - biase;
             if (temp_y > 0)
                cent_y  = temp_y - fmod(temp_y, Sy)  + Sy / 2 + biase;
             else
                 cent_y  = temp_y - fmod(temp_y, Sy)  - Sy / 2 + biase;

             if (it.z > 0)
                 cent_z  = it.z - fmod(it.z, Sz)  + Sz / 2;
             else
                 cent_z  = it.z - fmod(it.z, Sz)  - Sz / 2;


             //cout << "cent_x: " << cent_x << " cent_y " << cent_y << "  cent_z " << cent_z << endl;

             double begin_x = cent_x - Sx/2;
             double end_x   = cent_x + Sx/2;

             double begin_y = cent_y - Sy/2;
             double end_y   = cent_y + Sy/2;

             double begin_z = cent_z - Sz/2;
             double end_z   = cent_z + Sz/2;
             //cout << "begin_x " << begin_x << " end_x " << end_x << endl;
             cube_float_bin_vec[next_empty_id].Cent_x = cent_x;
             cube_float_bin_vec[next_empty_id].Cent_y = cent_y;
             cube_float_bin_vec[next_empty_id].Cent_z = cent_z;

             cube_float_bin_vec[next_empty_id].begin_x = begin_x;
             cube_float_bin_vec[next_empty_id].begin_y = begin_y;
             cube_float_bin_vec[next_empty_id].begin_z = begin_z;

             cube_float_bin_vec[next_empty_id].end_x = end_x;
             cube_float_bin_vec[next_empty_id].end_y = end_y;
             cube_float_bin_vec[next_empty_id].end_z = end_z;

             cube_float_bin_vec[next_empty_id].used = true;

             cube_float_bin_vec[next_empty_id].points->points.push_back(it);
             next_empty_id++;
            cout <<"**************&&&&&next_empty_id: " << next_empty_id <<  " cube messgae " << cent_x << " " << cent_y << " " << cent_z << endl;
             break;

         }
      }
     // cout << "next_empty_id; "<< next_empty_id << endl;
      //cout << "inside cube_create_cnt" << cube_create_cnt << " cube_used_cnt " << cube_used_cnt <<  " sertion_points_cnt "<< sertion_points_cnt << endl;
  }
}


void Cube_float::put_points_in_occupyed_cube(void)
{
     for(auto it : points_in->points) {
         for (std::vector<Cube_namespace::cube_float_bin>::iterator it_bin = cube_float_bin_vec.begin();
              it_bin < cube_float_bin_vec.end(); it_bin++) {
             if (it_bin->used == true) {
                 if ((it.x > it_bin->begin_x) && (it.x < it_bin->end_x))
                     if ((it.y > it_bin->begin_y) && (it.y < it_bin->end_y))
                         if ((it.z > it_bin->begin_z) && (it.z < it_bin->end_z)) {
                             it_bin->points->push_back(it);
                             break;
                         }
             }
         }
     }
}
void Cube_float::put_intersection_points_in_occupyed_cube(void)
{
for(auto it : intesertion_plan_point->points) {
    for (std::vector<Cube_namespace::cube_float_bin>::iterator it_bin = cube_float_bin_vec.begin();
        it_bin < cube_float_bin_vec.end(); it_bin++) {
            if (it_bin->used == true) {
                if ((it.x > it_bin->begin_x) && (it.x < it_bin->end_x))
                    if ((it.y > it_bin->begin_y) && (it.y < it_bin->end_y))
                        if ((it.z > it_bin->begin_z) && (it.z < it_bin->end_z)) {
                             it_bin->points->push_back(it);
                            break;
                        }
            }
        }
    }
}
int Cube_float::get_bin_num_occupyed()
{
    int bin_cnt = 0;
    for (int i = 0; i < next_empty_id; i++) {
        if (cube_float_bin_vec[i].used == true)
           bin_cnt++;
    }
    return bin_cnt;
}

int Cube_float::get_pointsNum_of_occupyed_bin(void)
{
    int num = 0;
    for (int i = 0; i < next_empty_id; i++)
    {
        if(cube_float_bin_vec[i].used == true)
       num += cube_float_bin_vec[i].points->points.size();
    }
    return num;
}
void Cube_float::get_points_of_occupyed_bin(pcl::PointCloud<pcl::PointXYZINormal>::Ptr bin_cloud)
{
    for (int i = 0; i < next_empty_id; i++)
    {
        if(cube_float_bin_vec[i].used == true)
        *bin_cloud += *cube_float_bin_vec[i].points;
    }

}

double Cube_float::cal_compont(void)
{
    double radio_bin_all = 0;

   for (int i = 0; i < next_empty_id; i++)
   {
       cout << "inside: bin: " << i << " next empty id: " << next_empty_id << endl;
       if (cube_float_bin_vec[i].used == true)
       {
           Cube_namespace::id_stc id_stc = {0};
           cout << "cal_compont: points size : " << cube_float_bin_vec[i].points->points.size() << endl;
           for (pcl::PointCloud<pcl::PointXYZINormal>::iterator it = cube_float_bin_vec[i].points->points.begin();
                it < cube_float_bin_vec[i].points->points.end(); it++)
           {
               if (it->curvature == Cube_namespace::GROUND)
               {
                   id_stc.num_ground++;
               }
               else if (it->curvature == Cube_namespace::NO_GROUND)
               {
                   id_stc.num_no_ground++;
               }
               else if (it->curvature == Cube_namespace::TOP)
               {
                   id_stc.num_top ++;
               }

               id_stc.num_all++;
           }

           if (id_stc.num_all > 0)
           {
               double temp1 = (double) (id_stc.num_ground + id_stc.num_top);
               double temp2 = (double) (id_stc.num_all);
               cube_float_bin_vec[i].compont = id_stc;
               cube_float_bin_vec[i].compont.radio =1.0 -  2.0 * fabs(temp1 / temp2 - 0.5);
               //cout << "cube compont_radio: " << cube_float_bin_vec[i].compont_radio << endl;
               radio_bin_all += cube_float_bin_vec[i].compont.radio;
           }
       }

   }

   radio_bin_all = radio_bin_all / (double)next_empty_id ;
   return radio_bin_all;
}

void Cube_float::clear_occupyed_bin_low_radio(double th)
{
    std::vector<Cube_namespace::cube_float_bin>::iterator it_bin_to_empty_id = cube_float_bin_vec.begin() + next_empty_id;
    for (  std::vector<Cube_namespace::cube_float_bin>::iterator it = cube_float_bin_vec.begin();
        it < it_bin_to_empty_id; it++)
    {
        if(it->used == true)
        if (it->compont.radio < th)
        {
                clear_cube(it);
        }
    }
}
void Cube_float::clear_occupyed_bin_low_num(int points_num)
{
    std::vector<Cube_namespace::cube_float_bin>::iterator it_bin_to_empty_id = cube_float_bin_vec.begin() + next_empty_id;
    for (  std::vector<Cube_namespace::cube_float_bin>::iterator it = cube_float_bin_vec.begin();
           it < it_bin_to_empty_id; it++)
    {
        if(it->used==true)
        if (it->compont.num_no_ground < points_num)
            clear_cube(it);
        else
        if((it->compont.num_ground > points_num) ||(it->compont.num_top > points_num)){}
            else
            clear_cube(it);

    }
}

void Cube_float::clear_cube(std::vector<Cube_namespace::cube_float_bin>::iterator &it) {
    if(it->used == true)
    {
        it->points->points.clear();
        it->Cent_x = 0;
        it->Cent_y = 0;
        it->Cent_z = 0;
        it->used = false;
        it->begin_x = 0;
        it->end_x = 0;
        it->begin_y = 0;
        it->begin_y = 0;
        it->end_z = 0;
        it->end_z = 0;
        it->compont.radio = 0;
    }
}

Cube_cla::Cube_cla(double Cube_len, int Cube_num):Cube_len(Cube_len),
                                                  Cube_num_x(Cube_num),Cube_num_y(Cube_num),Cube_num_z(Cube_num) {
    Cube_numbs = Cube_num_x*Cube_num_y*Cube_num_z;

    begin_x = 0;                             end_x = Cube_num_x * Cube_len;
    begin_y = -(Cube_num_y * Cube_len) / 2;  end_y = (Cube_num_y * Cube_len) / 2;
    begin_z = -(Cube_num_z * Cube_len) / 2;  end_z = (Cube_num_z * Cube_len) / 2;

    for (int i = 0; i < Cube_num_x; i++)
        for (int j = 0; j < Cube_num_y; j++)
            for (int k = 0; k < Cube_num_z; k++)
            {
                pcl::PointCloud<PointType>::Ptr tempPoints(new pcl::PointCloud<PointType>());
                cube_vec.push_back(tempPoints);
            }
};

void Cube_cla::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr points_in)
{
    //cout << "inside Put_points: points size: " << points_in->size() << endl;
    for (auto it:cube_vec) //新的一帧点云，则清除之前的
    {
        it->clear();
    }
    std::vector<Cube_namespace::id_stc>().swap(id_vec);
    std::vector<Cube_namespace::COMPONT_RADIO>().swap(compont_radio_vec);

    for (int i = 0; i < points_in->size(); i++)
    {
        //std::cout << points_in->points[i].x  << endl;
        if (points_in->points[i].x < begin_x || points_in->points[i].x >= end_x) continue;
        if (points_in->points[i].y < begin_y || points_in->points[i].y >= end_y) continue;
        if (points_in->points[i].z < begin_z || points_in->points[i].z >= end_z) continue;

        double temp_x = points_in->points[i].x + begin_x;
        double temp_y = points_in->points[i].y + end_y;
        double temp_z = points_in->points[i].z + end_z;

        int id_x = 0, id_y = 0, id_z = 0;
        //std::cout << "tmep_x" << temp_x << endl;
        id_x = int((temp_x - fmod(temp_x, Cube_len)) / Cube_len);
        id_y = int((temp_y - fmod(temp_y, Cube_len)) / Cube_len);
        id_z = int((temp_z - fmod(temp_z, Cube_len)) / Cube_len);

        if (temp_x < Cube_len) id_x = 0;
        if (temp_y < Cube_len) id_y = 0;
        if (temp_z < Cube_len) id_z = 0;

        //std::cout << "id_x" << id_x << "y: " << id_y << "z" << id_z <<endl;
        int id = id_x*(Cube_num_y * Cube_num_z) + id_y*(Cube_num_z) + id_z;
        //std::cout << "id: " << id << endl;
        //std::cout << "cube_vec size: " << cube_vec.size() << endl;
        cube_vec[id]->points.push_back(points_in->points[i]);
    }
/*
    for (auto it:cube_vec)
    {
        cout << "inside: cube_vec points size: " << it->points.size() << endl;
    }
    */
}

std::vector<int> Cube_cla::get_id_vec(void)
{
    std::vector<int>  temp_id_vec;
    for (int i = 0; i < cube_vec.size(); i++)
    {
        if( cube_vec[i]->points.size() > 10)
        {
            temp_id_vec.push_back(i);
        }
    }
    return temp_id_vec;
}

std::vector<Cube_namespace::id_stc> Cube_cla::num_cube(int num_points_th)
{
   // cout << "inside: cube_vec size: " << cube_vec.size() << endl;
    for (int i = 0; i < cube_vec.size(); i++)
    {
        Cube_namespace::id_stc id_stc = {0};
        Cube_namespace::COMPONT_RADIO comp_radio = {0};
    //    cout << "inside: cube_vec points size: " << cube_vec[i]->size() << endl;
        for (pcl::PointCloud<PointType>::iterator it = cube_vec[i]->begin(); it < cube_vec[i]->end(); it++)
        {
           if (it->curvature == Cube_namespace::GROUND)
           {
             id_stc.num_ground++;
           }
           else if (it->curvature == Cube_namespace::NO_GROUND)
           {
               id_stc.num_no_ground++;
           }
           else if (it->curvature == Cube_namespace::TOP)
           {
              id_stc.num_top ++;
           }

           id_stc.num_all++;
        }
     //   cout << "inside: id_stc num_all: " << id_stc.num_all << endl;
        if (id_stc.num_all > num_points_th)
        {
            id_stc.id = i;
            comp_radio.id = i;
            comp_radio.radio = double( id_stc.num_ground + id_stc.num_top ) / double(id_stc.num_all);

            id_vec.push_back(id_stc);
            compont_radio_vec.push_back(comp_radio);

        }
      //  cout << "inside: num_all: " << comp_radio.radio << endl;
    }
    //cout << "inside: id_vec size: " << id_vec.size() << endl;
    //cout << "inside: compont_radio_vec size: " << compont_radio_vec.size() << endl;
    return id_vec;
}

std::vector<Cube_namespace::COMPONT_RADIO> Cube_cla::get_compont_radio(double radio_th)
{
    std::vector<Cube_namespace::COMPONT_RADIO> compont_radio_temp;

    for (auto it : compont_radio_vec)
    {
       // cout << "inside: it radio: " << it.radio << endl;
        if ((it.radio > radio_th) && (it.radio < (1.0 - radio_th)))
        {
           compont_radio_temp.push_back(it);
        }
    }
   // cout << "inside: compoint_radio_temp size: " << compont_radio_temp.size() << endl;
    return compont_radio_temp;
}

Norm_est::Norm_est(double search_radio):search_radio(search_radio)
{
    points_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    normal_temp.reset(new pcl::PointCloud<pcl::Normal>);

    ne_tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());
    ne.reset(new pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal>());
    ne->setSearchMethod(ne_tree);
    ne->setRadiusSearch(search_radio);
    ne->setViewPoint(0,0,0);
}

Norm_est::Norm_est(int search_k):search_k(search_k)
{
    points_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    normal_temp.reset(new pcl::PointCloud<pcl::Normal>);

    ne_tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());
    ne.reset(new pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal>());
    ne->setSearchMethod(ne_tree);
    ne->setKSearch(search_k);
    ne->setViewPoint(0,0,0);
}

void Norm_est::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in,
                          pcl::PointCloud<pcl::Normal>::Ptr points_normal) {

    points_temp->clear();
    normal_temp->clear();
    /*这里不重新指定新的地址kdtree会出现保留前一帧的情况*/
    ne_tree.reset(new pcl::search::KdTree<pcl::PointXYZINormal>());
    ne->setSearchMethod(ne_tree);
    ne->setKSearch(search_k);
    ne->setViewPoint(0,0,0);

    ne->setInputCloud(points_in);
    ne->compute(*points_normal);
//std::cout << "points_In size: " << points_in->points.size() << endl;
    std::vector<int> temp_vec;
    pcl::removeNaNNormalsFromPointCloud(*points_normal, *normal_temp, temp_vec);
 //   std::cout << "inside temp_vec size: " << temp_vec.size() << endl;
  //  std::cout << "points_normal size: " << points_normal->points.size()
   // << " " << "normal temp siez: " << normal_temp->points.size() << endl;
    int cnt;
    for (auto it: temp_vec) {
        points_temp->push_back(points_in->points[it]);
        points_temp->points[cnt].normal_x = points_normal->points[it].normal_x;
        points_temp->points[cnt].normal_y = points_normal->points[it].normal_y;
        points_temp->points[cnt].normal_z = points_normal->points[it].normal_z;

        points_temp->points[cnt].intensity = points_normal->points[it].curvature;
        cnt++;
    }

    *points_in = *points_temp;
    *points_normal = *normal_temp;
}
void Norm_est::Put_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_out,
                          pcl::PointCloud<pcl::Normal>::Ptr points_normal){

    ne->setInputCloud(points_in);
    ne->compute(*points_normal);

    std::vector<int> temp_vec;
    pcl::removeNaNNormalsFromPointCloud(*points_normal, *points_normal, temp_vec);
    for (auto it:temp_vec)
    {
        points_out->push_back(points_in->points[it]);
    }
}

void PointHandle::removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, float thres) {

    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size()) {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void PointHandle::removeFarestPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, float thres) {

    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z > thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size()) {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

Livox_feature::Livox_feature()
{
    matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    /*
    matA1.create( 3, 3, CV_32F);
    matD1.create( 3, 3, CV_32F);
    matV1.create( 3, 3, CV_32F);
     */
}

bool Livox_feature::plane_judge(const std::vector<PointType>& point_list,const int plane_threshold)
{
    int num = point_list.size();
    float cx = 0;
    float cy = 0;
    float cz = 0;
    for (int j = 0; j < num; j++) {
        cx += point_list[j].x;
        cy += point_list[j].y;
        cz += point_list[j].z;
    }
    cx /= num;
    cy /= num;
    cz /= num;
    //mean square error
    float a11 = 0;
    float a12 = 0;
    float a13 = 0;
    float a22 = 0;
    float a23 = 0;
    float a33 = 0;
    for (int j = 0; j < num; j++) {
        float ax = point_list[j].x - cx;
        float ay = point_list[j].y - cy;
        float az = point_list[j].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
    }
    a11 /= num;
    a12 /= num;
    a13 /= num;
    a22 /= num;
    a23 /= num;
    a33 /= num;

    matA1.at<float>(0, 0) = a11;
    matA1.at<float>(0, 1) = a12;
    matA1.at<float>(0, 2) = a13;
    matA1.at<float>(1, 0) = a12;
    matA1.at<float>(1, 1) = a22;
    matA1.at<float>(1, 2) = a23;
    matA1.at<float>(2, 0) = a13;
    matA1.at<float>(2, 1) = a23;
    matA1.at<float>(2, 2) = a33;

    cv::eigen(matA1, matD1, matV1);
    if (matD1.at<float>(0, 0) > plane_threshold * matD1.at<float>(0, 1)) {
        return true;
    }
    else{
        return false;
    }
}
void Livox_feature::extract_feature(pcl::PointCloud<PointType> &laserCloudIn,
                     pcl::PointCloud<PointType> &surfPointsFlat,
                     pcl::PointCloud<PointType> &cornerPointsSharp)
{

        int cloudSize = laserCloudIn.points.size();

        //std::cout<<"DEBUG first cloudSize "<<cloudSize<<std::endl;

        if(cloudSize > 32000) cloudSize = 32000;

        int count = cloudSize;

        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        for (int i = 0; i < cloudSize; i++) {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            point.intensity = laserCloudIn.points[i].intensity;
            point.curvature = laserCloudIn.points[i].curvature;
            int scanID = 0;
            if (N_SCANS == 6) {
                scanID = (int)point.intensity;
            }
            laserCloudScans[scanID].push_back(point);
        }

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCANS; i++) {
            *laserCloud += laserCloudScans[i];
        }

        cloudSize = laserCloud->size();

        for (int i = 0; i < cloudSize; i++) {
            CloudFeatureFlag[i] = 0;
        }
        /*
         pcl::PointCloud<PointType> cornerPointsSharp;

         pcl::PointCloud<PointType> surfPointsFlat;

         pcl::PointCloud<PointType> laserCloudFull;
         */

        int debugnum1 = 0;
        int debugnum2 = 0;
        int debugnum3 = 0;
        int debugnum4 = 0;
        int debugnum5 = 0;

        int count_num = 1;
        bool left_surf_flag = false;
        bool right_surf_flag = false;
        Eigen::Vector3d surf_vector_current(0,0,0);
        Eigen::Vector3d surf_vector_last(0,0,0);
        int last_surf_position = 0;
        double depth_threshold = 0.1;


        //********************************************************************************************************************************************
        for (int i = 5; i < cloudSize - 5; i += count_num ) {
            float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                               laserCloud->points[i].y * laserCloud->points[i].y +
                               laserCloud->points[i].z * laserCloud->points[i].z);

            // if(depth < 2) depth_threshold = 0.05;
            // if(depth > 30) depth_threshold = 0.1;
            //left curvature
            float ldiffX =
                    laserCloud->points[i - 4].x + laserCloud->points[i - 3].x
                    - 4 * laserCloud->points[i - 2].x
                    + laserCloud->points[i - 1].x + laserCloud->points[i].x;

            float ldiffY =
                    laserCloud->points[i - 4].y + laserCloud->points[i - 3].y
                    - 4 * laserCloud->points[i - 2].y
                    + laserCloud->points[i - 1].y + laserCloud->points[i].y;

            float ldiffZ =
                    laserCloud->points[i - 4].z + laserCloud->points[i - 3].z
                    - 4 * laserCloud->points[i - 2].z
                    + laserCloud->points[i - 1].z + laserCloud->points[i].z;

            float left_curvature = ldiffX * ldiffX + ldiffY * ldiffY + ldiffZ * ldiffZ;

            if(left_curvature < 0.01){

                std::vector<PointType> left_list;

                for(int j = -4; j < 0; j++){
                    left_list.push_back(laserCloud->points[i+j]);
                }

                if( left_curvature < 0.001) CloudFeatureFlag[i-2] = Livox_namespace::flat; //surf point flag  && plane_judge(left_list,1000)

                left_surf_flag = true;
            }
            else{
                left_surf_flag = false;
            }

            //right curvature
            float rdiffX =
                    laserCloud->points[i + 4].x + laserCloud->points[i + 3].x
                    - 4 * laserCloud->points[i + 2].x
                    + laserCloud->points[i + 1].x + laserCloud->points[i].x;

            float rdiffY =
                    laserCloud->points[i + 4].y + laserCloud->points[i + 3].y
                    - 4 * laserCloud->points[i + 2].y
                    + laserCloud->points[i + 1].y + laserCloud->points[i].y;

            float rdiffZ =
                    laserCloud->points[i + 4].z + laserCloud->points[i + 3].z
                    - 4 * laserCloud->points[i + 2].z
                    + laserCloud->points[i + 1].z + laserCloud->points[i].z;

            float right_curvature = rdiffX * rdiffX + rdiffY * rdiffY + rdiffZ * rdiffZ;

            if(right_curvature < 0.01){
                std::vector<PointType> right_list;

                for(int j = 1; j < 5; j++){
                    right_list.push_back(laserCloud->points[i+j]);
                }
                if(right_curvature < 0.001 ) CloudFeatureFlag[i+2] = Livox_namespace::flat; //surf point flag  && plane_judge(right_list,1000)

                count_num = 4;
                right_surf_flag = true;
            }
            else{
                count_num = 1;
                right_surf_flag = false;
            }

            //surf-surf corner feature
            if(left_surf_flag && right_surf_flag){
                debugnum4 ++;

                Eigen::Vector3d norm_left(0,0,0);
                Eigen::Vector3d norm_right(0,0,0);
                for(int k = 1;k<5;k++){
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i-k].x-laserCloud->points[i].x,
                                                          laserCloud->points[i-k].y-laserCloud->points[i].y,
                                                          laserCloud->points[i-k].z-laserCloud->points[i].z);
                    tmp.normalize();
                    norm_left += (k/10.0)* tmp;
                }
                for(int k = 1;k<5;k++){
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i+k].x-laserCloud->points[i].x,
                                                          laserCloud->points[i+k].y-laserCloud->points[i].y,
                                                          laserCloud->points[i+k].z-laserCloud->points[i].z);
                    tmp.normalize();
                    norm_right += (k/10.0)* tmp;
                }

                //calculate the angle between this group and the previous group
                double cc = fabs( norm_left.dot(norm_right) / (norm_left.norm()*norm_right.norm()) );
                //calculate the maximum distance, the distance cannot be too small
                Eigen::Vector3d last_tmp = Eigen::Vector3d(laserCloud->points[i-4].x-laserCloud->points[i].x,
                                                           laserCloud->points[i-4].y-laserCloud->points[i].y,
                                                           laserCloud->points[i-4].z-laserCloud->points[i].z);
                Eigen::Vector3d current_tmp = Eigen::Vector3d(laserCloud->points[i+4].x-laserCloud->points[i].x,
                                                              laserCloud->points[i+4].y-laserCloud->points[i].y,
                                                              laserCloud->points[i+4].z-laserCloud->points[i].z);
                double last_dis = last_tmp.norm();
                double current_dis = current_tmp.norm();

                if(cc < 0.5 && last_dis > 0.05 && current_dis > 0.05 ){ // 角度不能太小
                    debugnum5 ++;
                    //CloudFeatureFlag[i] = 150;
                    CloudFeatureFlag[i] = Livox_namespace::sharp1; //100 和 150
                }
            }
        }
        for(int i = 5; i < cloudSize - 5; i ++){
            float diff_left[2];
            float diff_right[2];
            float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                               laserCloud->points[i].y * laserCloud->points[i].y +
                               laserCloud->points[i].z * laserCloud->points[i].z);

            for(int count = 1; count < 3; count++ ){
                float diffX1 = laserCloud->points[i + count].x - laserCloud->points[i].x;
                float diffY1 = laserCloud->points[i + count].y - laserCloud->points[i].y;
                float diffZ1 = laserCloud->points[i + count].z - laserCloud->points[i].z;
                diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

                float diffX2 = laserCloud->points[i - count].x - laserCloud->points[i].x;
                float diffY2 = laserCloud->points[i - count].y - laserCloud->points[i].y;
                float diffZ2 = laserCloud->points[i - count].z - laserCloud->points[i].z;
                diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
            }

            float depth_right = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
            float depth_left = sqrt(laserCloud->points[i - 1].x * laserCloud->points[i - 1].x +
                                    laserCloud->points[i - 1].y * laserCloud->points[i - 1].y +
                                    laserCloud->points[i - 1].z * laserCloud->points[i - 1].z);

            //outliers
            if( (diff_right[0] > 0.1*depth && diff_left[0] > 0.1*depth) ){
                debugnum1 ++;
                CloudFeatureFlag[i] = Livox_namespace::out_lier; //排除的点
                continue;
            }

            //break points
            if(fabs(diff_right[0] - diff_left[0])>0.1){
                if(diff_right[0] > diff_left[0]){

                    Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i-4].x-laserCloud->points[i].x,
                                                                  laserCloud->points[i-4].y-laserCloud->points[i].y,
                                                                  laserCloud->points[i-4].z-laserCloud->points[i].z);
                    Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                                   laserCloud->points[i].y,
                                                                   laserCloud->points[i].z);
                    double left_surf_dis = surf_vector.norm();
                    //calculate the angle between the laser direction and the surface
                    double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

                    std::vector<PointType> left_list;
                    double min_dis = 10000;
                    double max_dis = 0;
                    for(int j = 0; j < 4; j++){   //TODO: change the plane window size and add thin rod support
                        left_list.push_back(laserCloud->points[i-j]);
                        Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i-j].x-laserCloud->points[i-j-1].x,
                                                                      laserCloud->points[i-j].y-laserCloud->points[i-j-1].y,
                                                                      laserCloud->points[i-j].z-laserCloud->points[i-j-1].z);

                        if(j == 3) break;
                        double temp_dis = temp_vector.norm();
                        if(temp_dis < min_dis) min_dis = temp_dis;
                        if(temp_dis > max_dis) max_dis = temp_dis;
                    }
                    bool left_is_plane = plane_judge(left_list,100);

                    if(left_is_plane && (max_dis < 2*min_dis) && left_surf_dis < 0.05 * depth  && cc < 0.8){//
                        if(depth_right > depth_left){
                            CloudFeatureFlag[i] = Livox_namespace::sharp0;
                        }
                        else{
                            if(depth_right == 0) CloudFeatureFlag[i] = Livox_namespace::sharp0;
                        }
                    }
                }
                else{

                    Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i+4].x-laserCloud->points[i].x,
                                                                  laserCloud->points[i+4].y-laserCloud->points[i].y,
                                                                  laserCloud->points[i+4].z-laserCloud->points[i].z);
                    Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                                   laserCloud->points[i].y,
                                                                   laserCloud->points[i].z);
                    double right_surf_dis = surf_vector.norm();
                    //calculate the angle between the laser direction and the surface
                    double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

                    std::vector<PointType> right_list;
                    double min_dis = 10000;
                    double max_dis = 0;
                    for(int j = 0; j < 4; j++){ //TODO: change the plane window size and add thin rod support
                        right_list.push_back(laserCloud->points[i-j]);
                        Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i+j].x-laserCloud->points[i+j+1].x,
                                                                      laserCloud->points[i+j].y-laserCloud->points[i+j+1].y,
                                                                      laserCloud->points[i+j].z-laserCloud->points[i+j+1].z);

                        if(j == 3) break;
                        double temp_dis = temp_vector.norm();
                        if(temp_dis < min_dis) min_dis = temp_dis;
                        if(temp_dis > max_dis) max_dis = temp_dis;
                    }
                    bool right_is_plane = plane_judge(right_list,100);

                    if(right_is_plane && (max_dis < 2*min_dis) && right_surf_dis < 0.05 * depth && cc < 0.8){ //

                        if(depth_right < depth_left){
                            CloudFeatureFlag[i] = Livox_namespace::sharp0;
                        }
                        else{
                            if(depth_left == 0) CloudFeatureFlag[i] = Livox_namespace::sharp0;
                        }
                    }
                }
            }

            // break point select
            if(CloudFeatureFlag[i] == Livox_namespace::sharp0){
                debugnum2++;
                std::vector<Eigen::Vector3d> front_norms;
                Eigen::Vector3d norm_front(0,0,0);
                Eigen::Vector3d norm_back(0,0,0);
                for(int k = 1;k<4;k++){
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i-k].x-laserCloud->points[i].x,
                                                          laserCloud->points[i-k].y-laserCloud->points[i].y,
                                                          laserCloud->points[i-k].z-laserCloud->points[i].z);
                    tmp.normalize();
                    front_norms.push_back(tmp);
                    norm_front += (k/6.0)* tmp;
                }
                std::vector<Eigen::Vector3d> back_norms;
                for(int k = 1;k<4;k++){
                    Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i+k].x-laserCloud->points[i].x,
                                                          laserCloud->points[i+k].y-laserCloud->points[i].y,
                                                          laserCloud->points[i+k].z-laserCloud->points[i].z);
                    tmp.normalize();
                    back_norms.push_back(tmp);
                    norm_back += (k/6.0)* tmp;
                }
                double cc = fabs( norm_front.dot(norm_back) / (norm_front.norm()*norm_back.norm()) );
                if(cc < 0.8){
                    debugnum3++;
                }else{
                    CloudFeatureFlag[i] = 0;
                }

                continue;

            }
        }

        //push_back feature
        for(int i = 0; i < cloudSize; i++){
            //laserCloud->points[i].intensity = double(CloudFeatureFlag[i]) / 10000;
            float dis = laserCloud->points[i].x * laserCloud->points[i].x
                        + laserCloud->points[i].y * laserCloud->points[i].y
                        + laserCloud->points[i].z * laserCloud->points[i].z;
            float dis2 = laserCloud->points[i].y * laserCloud->points[i].y + laserCloud->points[i].z * laserCloud->points[i].z;
            float theta2 = std::asin(sqrt(dis2/dis)) / M_PI * 180;
            //std::cout<<"DEBUG theta "<<theta2<<std::endl;
            // if(theta2 > 34.2 || theta2 < 1){
            //    continue;
            // }
            //if(dis > 30*30) continue;

            if(CloudFeatureFlag[i] == Livox_namespace::flat){
                surfPointsFlat.push_back(laserCloud->points[i]);
                continue;
            }

            if(CloudFeatureFlag[i] == Livox_namespace::sharp0|| CloudFeatureFlag[i] == Livox_namespace::sharp1){
                cornerPointsSharp.push_back(laserCloud->points[i]);
            }
        }

/*
  std::cout<<"ALL point: "<<cloudSize<<" outliers: "<< debugnum1 << std::endl
            <<" break points: "<< debugnum2<<" break feature: "<< debugnum3 << std::endl
            <<" normal points: "<< debugnum4<<" surf-surf feature: " << debugnum5 << std::endl;
*/
/*
        sensor_msgs::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
        laserCloudOutMsg.header.frame_id = "zy_ka";
        pubLaserCloud.publish(laserCloudOutMsg);

        sensor_msgs::PointCloud2 cornerPointsSharpMsg;
        pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
        cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
        cornerPointsSharpMsg.header.frame_id = "zy_ka";
        pubCornerPointsSharp.publish(cornerPointsSharpMsg);

        sensor_msgs::PointCloud2 surfPointsFlat2;
        pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
        surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
        surfPointsFlat2.header.frame_id = "zy_ka";
        pubSurfPointsFlat.publish(surfPointsFlat2);
*/

    }

void Livox_feature::remove_sharp_in_ground(pcl::PointCloud<PointType> &cornerPointsSharp,
                            pcl::PointCloud<PointType> &groundPoints,
                            pcl::PointCloud<PointType> &cornerPointsSharpOut,
                            double radio, int k_th)
{
    pcl::search::KdTree<pcl::PointXYZINormal> kd_tree;
    kd_tree.setInputCloud(groundPoints.makeShared());
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    for (auto points : cornerPointsSharp.points)
    {
        std::vector<int>().swap(pointIdxRadiusSearch);
        std::vector<float>().swap(pointRadiusSquaredDistance);
        if (kd_tree.radiusSearch(points, radio, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            if (pointIdxRadiusSearch.size() < k_th)
            {
                cornerPointsSharpOut.push_back(points);
            }
        }
        else
        {
            cornerPointsSharpOut.push_back(points);
        }
    }


}
/*
std::vector<int> Cube_cla::get_id_num_ground_vec(int ground_num_th)
{
   for (int i = 0; i < cube_vec.size(); i++) {
       Cube_namespace::id_num id_num;
       for (pcl::PointCloud<PointType>::iterator it = cube_vec[i]->begin(); it < cube_vec[i]->end(); it++)
       {
           if (it->curvature == 0.0)
           {
               id_num.num++;
           }
       }

       if (id_num.num < ground_num_th)
       {
           id_num.id = i;
           id_vec_ground_num.push_back(id_num);
       }
   }
}
 */


