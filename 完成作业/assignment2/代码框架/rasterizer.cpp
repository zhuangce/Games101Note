// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
   
   // std::cout << "_v0: " <<_v[0]<< "_v1: " <<_v[1]<< "_v2: " <<_v[2]<< '\n';
    
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v1 (_v[0].x()-x ,_v[0].y()-y,1);
    Vector3f v2 (_v[1].x()-x ,_v[1].y()-y,1);
    Vector3f v3 (_v[2].x()-x ,_v[2].y()-y,1);
    

    float f1 = (v1.cross(v2)).dot(v2.cross(v3));
    float f2 = (v2.cross(v3)).dot(v3.cross(v1));
   float f3 = (v3.cross(v1)).dot(v1.cross(v2));
    f1 =f1*f2*f3 ;
   //  std::cout << "f1: " <<f1<< '\n';
    if(f1>0){
        
       return true;
    }else{
        false;
    }


}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
  //   std::cout << "c1: " << c1 << "c2: " << c2 << "c3: " << c3 << '\n';
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
     //std::cout << "rasterize_triangle " << '\n';
    auto v = t.toVector4();
   //  std::cout<< "size v " << v.size()<< '\n';
    //  for (size_t k = 0; k < v.size(); k++)
    // {
    //   std::cout << "rasterize_triangle v " << v[k] << '\n';
    // }
    //  std::cout << '\n';
    
    //   for (size_t k = 0; k < t.v->size(); k++)
    // {
    //   std::cout << " t.v " <<  t.v[k] << '\n';
    // }
   
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

   
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

  // TODO : Find out the bounding box of current triangle.
   // iterate through the pixel and find if the current pixel is inside the triangle
        float maxX =0;
        float minX = t.v[0].x();
          float maxY =0;
        float minY =t.v[0].y();
       
    for (size_t i = 0; i < 3; i++)
    {
       
        
        if(v[i].x()>maxX){
             maxX =v[i].x();
        }
        if(v[i].x()<minX){
             minX =v[i].x();
        }
      
        if(v[i].y()>maxY){
             maxY =v[i].y();
        }
        if(v[i].y()<minY){
             minY =v[i].y();
        }
         
    }
   //  std::cout << minX<< "-----" << maxX<< "-----"<<minY<< "-----" <<maxY<< "-----" << '\n';
    
    for (size_t i = floor(minX); i < maxX; i++)
    {
        
        auto x =i ;
       //  std::cout << "_X: " << x<< '\n';
        for (size_t j = floor(minY); j < maxY; j++)
        {
            auto y = j ;
           
           if(insideTriangle(x,y,t.v)){
   
                //auto [alpha, beta, gamma]  = computeBarycentric2D(x, y, t.v);
                auto t1 = computeBarycentric2D((float)x, (float)y, t.v);
               
                auto alpha = 1.0f;
               float beta =1.0f ;
                float gamma =1.0f ;
                std::tie(alpha,beta,gamma) = t1;
              // std::cout << "alpha: " << alpha << "beta: " << beta << "gamma: " << gamma << '\n';
              
              float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
              float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
            //    std::cout << "z_interpolated: " <<z_interpolated << "depth_buf[x*y]" << depth_buf[x*y]<< '\n';
                if(z_interpolated<depth_buf[x*y]){
                  depth_buf[x*y] = z_interpolated;
                   set_pixel(Eigen::Vector3f{(float)x,(float)y,1.0f},t.getColor());
                }                
            }
        }
    }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
   //  std::cout << "1: " << point[0]<<"2: " << point[1]<<"3: " << point[2] << '\n';
    // std::cout << "alpha: " << color[0]<<"beta: " << color[1]<<"gamma: " << color[2] << '\n';
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on