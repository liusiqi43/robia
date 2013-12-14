#include <mirage.h>
#include <cstdlib>
#include <sstream>
#include <dye.h>

typedef mirage::img::Coding<mirage::colorspace::RGB_24>::Frame ImageRGB;

class TestParam {
public:
  static double        reference;
  static double        colorIndex(void) {return reference;}
  static double        tolerance(void)  {return .3;}
  static unsigned char darkThreshold(void){return 50;}
};

double TestParam::reference=0;

typedef dye::Test<TestParam,ImageRGB::value_type> Test;

typedef mirage::algo::Labelizer<ImageRGB,Test,
                                mirage::algo::labelNone
                                | mirage::algo::labelBoundingBox> Labelizer;


int main(int argc, char* argv[]) {
  if(argc != 3) {
    std::cout << "Usage : " << argv[0] << " <input.jpg> <color-ref in [0,6]>" << std::endl;
    return 0;
  }

  ImageRGB input;
  ImageRGB display;
  Labelizer labelizer;
  TestParam::reference = atof(argv[2]);

  mirage::img::JPEG::read(input,argv[1]);

  mirage::colorspace::RGB_24 ref;
  dye::rgbOf(TestParam::reference,ref);
  dye::mask(input,display,Test(),
	    ref,mirage::colorspace::RGB_24(0,0,0));
  std::ostringstream filename;
  filename << "filter-" << TestParam::reference << ".jpg";
  mirage::img::JPEG::write(display,filename.str(),100);
  std::cout << "\"" << filename.str() << "\" generated." << std::endl;


  labelizer.neighborhoodSurround(); 
  labelizer(input);

  mirage::img::Line<ImageRGB> line;
  line << input;

  for(unsigned int l=1;l<=labelizer.nb_labels;++l)  {
    const Labelizer::BoundingBox& box = labelizer.boundingBox(l);
    mirage::img::Coordinate A,B,C,D;
    A = box.min();
    C = box.max();
    B(C[0],A[1]);
    D(A[0],C[1]);
    try {
      line(A,B,false,false);
      line = ImageRGB::value_type(255,255,0);
      line(B,C,false,false);
      line = ImageRGB::value_type(255,255,0);
      line(C,D,false,false);
      line = ImageRGB::value_type(255,255,0);
      line(D,A,false,false);
      line = ImageRGB::value_type(255,255,0);
    }
    catch(mirage::Exception::Line& e) {}
  }

  std::ostringstream filename2;
  filename2 << "labels-" << TestParam::reference << ".ppm";
  mirage::img::PPM::write(input,filename2.str());
  std::cout << "\""<< filename2.str() << "\" generated." << std::endl;
  

  return 0;
}
