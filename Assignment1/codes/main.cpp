#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Core>
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

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model <<
    std::cos(rotation_angle / 180 * M_PI), -std::sin(rotation_angle/ 180 * M_PI), 0, 0,
    std::sin(rotation_angle / 180 * M_PI), std::cos(rotation_angle/ 180 * M_PI), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f perspToOrthoMatrix = Eigen::Matrix4f::Identity();
    perspToOrthoMatrix << zNear, 0, 0, 0,
    0, zNear, 0 ,0,
    0, 0, zNear+zFar, -zFar*zNear,
    0, 0, 1, 0;

    Eigen::Matrix4f orthoMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    
    float t = zNear * std::tan(eye_fov / 2 / 180 * M_PI);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    m1 << 2/(r-l), 0, 0, 0,
    0, 2/(t-b), 0, 0,
    0, 0, 2/(zNear-zFar), 0,
    0, 0, 0, 1;

    m2 << 1, 0, 0, -(l+r)/2,
    0, 1, 0, -(b+t)/2,
    0, 0, 1, -(zNear+zFar)/2,
    0, 0, 0, 1;

    orthoMatrix = m1*m2;
    projection = orthoMatrix*perspToOrthoMatrix;

    return projection;
}

//Enhance Part

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    float arc = angle / 180 * M_PI;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();    

    Eigen::Matrix3f transformMatrix = Eigen::Matrix3f::Identity();
    transformMatrix << 0, -axis.z(), axis.y(),  
    axis.z(), 0, -axis.x(),
    -axis.y(), axis.x(), 0;

    Eigen::Matrix3f rotationMatrix = std::cos(arc) * I + (1 - std::cos(arc)) * axis * axis.transpose() + std::sin(arc) * transformMatrix;

    Eigen::Matrix4f wrapMatrix = Eigen::Matrix4f::Identity();
    wrapMatrix << rotationMatrix.row(0), 0,
    rotationMatrix.row(1), 0,
    rotationMatrix.row(2), 0,
    0, 0, 0, 1;

    return wrapMatrix;
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
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f axis = {0, 2, 5};   //enhance part needed

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    int activeEnhancePart = 0;  //active enhance function - getRotation()

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (!activeEnhancePart) {
            r.set_model(get_model_matrix(angle));
        } else {
            r.set_model(get_rotation(axis, angle));
        }
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

        if (!activeEnhancePart) {
            r.set_model(get_model_matrix(angle));
        } else {
            r.set_model(get_rotation(axis, angle));
        }
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
