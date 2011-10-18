#define BOOST_TEST_MODULE DenseStereoTest 
#include <boost/test/included/unit_test.hpp>

#include <vizkit/QtThreadedWidget.hpp>
#include <vizkit/QVisualizationTestWidget.hpp>
#include <vizkit/DistanceImageVisualization.hpp>
#include <iostream>

BOOST_AUTO_TEST_CASE( viz_test ) 
{
    std::cout << "Testing Dense Stereo Image" << std::endl;
    QtThreadedWidget<
	vizkit::QVisualizationTestWidget<
	    vizkit::DistanceImageVisualization, 
	    base::samples::DistanceImage> > app;
    app.start();
    std::cout << "Close the visualization window to abort this test." << std::endl;

    const size_t width = 640, height = 480;

    base::samples::DistanceImage image;
    image.width = width;
    image.height = height;
    image.scale_x = 1.0/width;
    image.scale_y = 1.0/width;
    image.center_x = 0.5;
    image.center_y = 0.5*height/width;
    image.data.resize(width*height);

    for( int i=0; i<1000 && app.isRunning(); i++ )
    {
        double r = i/100.0;
	for( size_t x=0; x<width; x++ )
	{
	    for( size_t y=0; y<height; y++ )
	    {
		float px = x / static_cast<float>(width);
		float py = y / static_cast<float>(width); 

		float d = sin(px + r) + cos(py);
		image.data[width*y+x] = d;
	    }
	}

        app.getWidget()->updateData( image );
        usleep( 10000 );
    }
}
