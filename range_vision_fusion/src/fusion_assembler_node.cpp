#include "range_vision_fusion/fusion_assembler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  ROSFusionAssemblerApp app;

  app.Run();

  return 0;
}
