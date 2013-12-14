#include <mirage.h>
#include <dye.h>

typedef mirage::img::Coding<mirage::colorspace::RGB_24>::Frame ImageRGB;
typedef mirage::img::Coding<double>::Frame                     ImageDouble;

#define DARK_RGB_COMPONENT 50

int main(int argc, char* argv[]) {
  if(argc != 2) {
    std::cout << "Usage : " << argv[0] << " <input.jpg>" << std::endl;
    return 0;
  }

  ImageRGB display;
  ImageRGB input;
  ImageDouble indexes;
  ImageDouble::pixel_type iter,end;
  ImageRGB::pixel_type    rgb;

  mirage::img::JPEG::read(input,argv[1]);

  indexes.resize(mirage::img::Coordinate(500,500));
  double factor = indexes._dimension[0]/6.0;
  for(iter=indexes.begin(),end=indexes.end();
      iter != end;
      ++iter)
    *iter = (!iter)[0]/factor;
  mirage::SubFrame<ImageDouble> square(indexes,
				       indexes.center(),
				       mirage::img::Coordinate(20,20));
  square = dye::darkIndex;

  display.resize(indexes._dimension);
  for(iter=indexes.begin(),end=indexes.end(),rgb=display.begin();
      iter != end;
      ++iter,++rgb)
    dye::rgbOf(*iter,*rgb);
  mirage::img::JPEG::write(display,"colors.jpg",100);
  std::cout << "\"colors.jpg\" generated." << std::endl;

  dye::index(input,indexes,
	     DARK_RGB_COMPONENT);
  display.resize(indexes._dimension);
  for(iter=indexes.begin(),end=indexes.end(),rgb=display.begin();
      iter != end;
      ++iter,++rgb)
    dye::rgbOf(*iter,*rgb);
  mirage::img::JPEG::write(display,"index.jpg",100);
  std::cout << "\"index.jpg\" generated." << std::endl;

  std::map<double,double> index_histogram;
  dye::indexHistogram(input,index_histogram,DARK_RGB_COMPONENT,100);
  dye::plot(index_histogram,"histo.pl");
  std::cout << "\"histo.pl\" generated." << std::endl;


  return 0;
}
