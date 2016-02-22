#include "imageFromDisplay.h"

ImageFromDisplay::ImageFromDisplay() {}

cv::Mat ImageFromDisplay::getImage()
{
    std::vector<std::uint8_t> pixels;
    int height, width, bpp;
    height = width = bpp = 0;
    captureImage(pixels, width, height);
    if (width && height) return cv::Mat(height, width, CV_8UC4, &pixels[0]);
    else return cv::Mat();
}

void ImageFromDisplay::captureImage(std::vector<uint8_t>& pixels, int& width,
				    int& height)
{
    Display* display = XOpenDisplay(nullptr);
    Window root = DefaultRootWindow(display);

    XWindowAttributes attributes = {0};
    XGetWindowAttributes(display, root, &attributes);

    width = attributes.width;
    height = attributes.height;

    XImage* img =
	XGetImage(display, root, 0, 0, width, height, AllPlanes, ZPixmap);
    pixels.resize(width * height * 4);

    memcpy(&pixels[0], img->data, pixels.size());

    XFree(img);
    XCloseDisplay(display);
}
