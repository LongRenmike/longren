
#include <opencv2/opencv.hpp>
#include <vector>

void image_main();

extern cv::Mat raw_frame;
extern cv::Mat grayFrame;
extern cv::Mat binarizedFrame;
extern cv::Mat morphologyExFrame;
extern cv::Mat track;

extern std::vector<int> left_line;
extern std::vector<int> right_line;
extern std::vector<int> mid_line;

extern int line_tracking_height;
extern int line_tracking_width;

extern int yellowAreaCount;
extern bool zebraLineDetected;

bool detectZebraCrossing(const cv::Mat& gray);
int detectYellowBlocks(const cv::Mat& inputImage);
bool detectRedBlock(const cv::Mat& inputImage);
int detectWhiteBlocks(const cv::Mat& inputImage);

void findTrackByLongestWhiteColumn(const cv::Mat& binaryImg,
                                   std::vector<int>& left_line,
                                   std::vector<int>& right_line,
                                   std::vector<int>& mid_line);
