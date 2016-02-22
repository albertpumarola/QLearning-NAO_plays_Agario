#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>

class ImageFromDisplay
{
public:
    ImageFromDisplay();
    cv::Mat getImage();

private:
    void captureImage(std::vector<uint8_t>& pixels, int& width,
				    int& height);
};
