#include "armor_board_processor/armor_board_processor.h"
#include "rclcpp/logging.hpp"

using namespace std;
using namespace cv;

Mat ArmorProcessor::preprocess_image(const Mat& frame) 
{
    Mat hsv, armor;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(100, 100, 100), Scalar(130, 255, 255), armor);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    for(int i=0;i<2;i++)
    { 
        dilate(armor,armor,kernel);
    }
    for(int i=0;i<2;i++)
    {
        erode(armor,armor,kernel);
    }
    morphologyEx(armor,armor,MORPH_CLOSE,kernel);
    morphologyEx(armor,armor,MORPH_OPEN,kernel);
    GaussianBlur(armor, armor, Size(3,3), 2, 2);
    return armor;
}

vector<RotatedRect> ArmorProcessor::detect_armors(const Mat& frame) 
{
    vector<RotatedRect> armors;
    vector<vector<Point>> contours;
    findContours(frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) 
    {
        if (contourArea(contour) < 50) 
        {
            continue;
        }
        RotatedRect rect = minAreaRect(contour);
        float ratio = max(rect.size.width, rect.size.height) / min(rect.size.width, rect.size.height);
        if (ratio > 3 && ratio < 8) 
        {
            armors.push_back(rect);
        }
    }
    return armors;
}

RotatedRect ArmorProcessor::select_target_armor(const vector<RotatedRect>& armors) 
{
    int max_idx = 0;
    float max_area = armors[0].size.area();
    for (size_t i = 1; i < armors.size(); ++i) 
    {
        float area = armors[i].size.area();
        if (area > max_area) 
        {
            max_area = area;
            max_idx = i;
        }
    }
    return armors[max_idx];
}
