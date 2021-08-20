#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotationZ;
    float angle = rotation_angle * MY_PI / 180;
    rotationZ<<cos(angle),-sin(angle),0,0,
               sin(angle), cos(angle),0,0,
               0,0,1,0,
               0,0,0,1;
    model = rotationZ * model;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    //Eigen::Matrix4f::Identity() 初始化为单位矩阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    P2O<<zNear,0,0,0,
         0,zNear,0,0,
         0,0,zNear+zFar,-zNear*zFar,
         0,0,1.0,0;

    float halfEyeAngleRad=eye_fov/180*MY_PI;
    float y_top = -zNear*tan(halfEyeAngleRad);
    float y_bottom = -y_top;
    float x_left = -y_top*aspect_ratio;
    float x_right = -x_left;

    Eigen::Matrix4f scaleMat = Eigen::Matrix4f::Identity();
    scaleMat << 2/(x_right - x_left),0,0,0,
                0,2/(y_top - y_bottom),0,0,
                0,0,2/(zNear - zFar),0,
                0,0,0,1;

    Eigen::Matrix4f translateMat = Eigen::Matrix4f::Identity();
    translateMat << 1, 0, 0, 0,              
                    0, 1, 0, 0,                 
                    0, 0, 1, -(zNear+zFar)/2,
                    0, 0, 0, 1;

    projection = translateMat * scaleMat * P2O * projection;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

Eigen::Matrix4f get_rotation(float angle, Vector3f axis)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f temp = Eigen::Matrix3f::Identity();
    float ag=angle/180*MY_PI;
    Eigen::Matrix3f N, tr;
    N << 0,-axis(2),axis(1),
         axis(2),0,-axis(0),
         -axis(1),axis(0),0;

    tr=cos(ag)*temp+(1-cos(ag))*axis*axis.adjoint()+sin(ag)*N;
    model <<  tr(0,0), tr(0,1), tr(0,2), 0,
            tr(1,0), tr(1,1), tr(1,2), 0,
            tr(2,0), tr(2,1), tr(2,2), 0,
            0, 0, 0, 1;
    return model;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
