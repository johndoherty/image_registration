/*
 * ONIVideoInput.cpp
 *
 *  Created on: Nov 19, 2014
 *      Author: john
 */

// OpenNI
#define THROW_IF_FAILED(retVal) {if (retVal != XN_STATUS_OK) throw xnGetStatusString(retVal);}

using namespace std;
using namespace cv;


void run(const std::string& codecName,
           const std::string& inputFile, const std::string& outputFile, bool depthAsPng = false)
  {
    // TODO For the latest version of OpenCV, you may use HighGUI instead of using OpenNI
    // assumed that nframes, picture size for depth and images is the same
    xn::Context context;
    THROW_IF_FAILED(context.Init());

    xn::Player player;
    THROW_IF_FAILED(context.OpenFileRecording(inputFile.c_str(), player));
    THROW_IF_FAILED(player.SetRepeat(false));

    xn::ImageGenerator imageGen;
    THROW_IF_FAILED(imageGen.Create(context));
    XnPixelFormat pixelFormat = imageGen.GetPixelFormat();
    if (pixelFormat != XN_PIXEL_FORMAT_RGB24) {
      THROW_IF_FAILED(imageGen.SetPixelFormat(XN_PIXEL_FORMAT_RGB24));
    }
    xn::ImageMetaData xImageMap2;
    imageGen.GetMetaData(xImageMap2);
    XnUInt32 fps = xImageMap2.FPS();
    XnUInt32 frame_height = xImageMap2.YRes();
    XnUInt32 frame_width = xImageMap2.XRes();

    xn::DepthGenerator depthGen;
    depthGen.Create(context);
    XnUInt32 nframes;
    player.GetNumFrames(depthGen.GetName(), nframes);

    std::string outputFileImg, outputFileDepth;
    getOutputFileNames(outputFile, outputFileImg, outputFileDepth);

    printResume(nframes, codecName, inputFile, outputFileImg, depthGen);

    //check permissions to write in the current directory
    //fs::path currentFolder("./");
    //fs::file_status st = fs::status(currentFolder);
    //std::cout << (st.permissions() & fs::all_all) << std::endl;

    cv::VideoWriter imgWriter(outputFileImg, m_codecName2Code(codecName), fps, cvSize(frame_width, frame_height), 1);
    cv::VideoWriter depthWriter;
    if (!depthAsPng)
      depthWriter.open(outputFileDepth, m_codecName2Code(codecName), fps, cvSize(frame_width, frame_height), 1);

    fs::path folderForDepthImages = getDepthFolderName(outputFileImg);
    if (depthAsPng)
    {
      if (fs::exists(folderForDepthImages) && !fs::is_directory(folderForDepthImages))
      {
        throw "Cannot create a directory because file with the same name exists. Remove it and try again";
      }
      if (!fs::exists(folderForDepthImages))
      {
        fs::create_directory(folderForDepthImages);
      }
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    THROW_IF_FAILED(context.StartGeneratingAll());

    size_t outStep = nframes > 10 ? nframes / 10 : 1;

    try
    {
      for(size_t iframe = 0; iframe < nframes; ++iframe)
      {
        if ( iframe % outStep == 0 )
            std::cout << iframe << "/" << nframes << std::endl;

        // save image
        THROW_IF_FAILED(imageGen.WaitAndUpdateData());
        xn::ImageMetaData xImageMap;
        imageGen.GetMetaData(xImageMap);
        XnRGB24Pixel* imgData = const_cast<XnRGB24Pixel*>(xImageMap.RGB24Data());
        cv::Mat image(frame_height, frame_width, CV_8UC3, reinterpret_cast<void*>(imgData));

        cv::cvtColor(image, image, CV_BGR2RGB); // opencv image format is BGR
        imgWriter << image.clone();

        // save depth
        THROW_IF_FAILED(depthGen.WaitAndUpdateData());
        xn::DepthMetaData xDepthMap;
        depthGen.GetMetaData(xDepthMap);
        XnDepthPixel* depthData = const_cast<XnDepthPixel*>(xDepthMap.Data());
        cv::Mat depth(frame_height, frame_width, CV_16U, reinterpret_cast<void*>(depthData));

        if (!depthAsPng)
        {
  #if (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4) || CV_MAJOR_VERSION > 2
          HistogramNormalizer::run(depth);
          cv::Mat depthMat8UC1;
          depth.convertTo(depthMat8UC1, CV_8UC1);

          // can be used for having different colors than grey
          cv::Mat falseColorsMap;
          cv::applyColorMap(depthMat8UC1, falseColorsMap, cv::COLORMAP_AUTUMN);
          depthWriter << falseColorsMap;
  #else
        throw "saving depth in avi file is not supported for opencv 2.3 or earlier. please use option --depth-png=yes";
  #endif
        }
        else
        {
          // to_string is not supported by gcc4.7 so I don't use it here
          //std::string imgNumAsStr = std::to_string(imgNum);
          std::stringstream ss;
          ss << folderForDepthImages.string() << "/depth-" << iframe << ".png";

          cv::imwrite(ss.str(), depth, compression_params);
        }
      }
    }
    catch(...)
    {
      context.StopGeneratingAll();
      context.Release();
      throw;
    }

    context.StopGeneratingAll();
    context.Release();
  }











ONIVideoInput::ONIVideoInput(string filename) {
	frameCount = 0;
	hasFrame = true;
}



ONIVideoInput::~ONIVideoInput() {
	currentFrame.release();
	inputVideo.release();
}

void ONIVideoInput::getCurrentFrame(Mat &output) {

}

bool ONIVideoInput::hasNextFrame() {
	return hasFrame;
}

int ONIVideoInput::getCurrentFrameCount() {
	return frameCount;
}


