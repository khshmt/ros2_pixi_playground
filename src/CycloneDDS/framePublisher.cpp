#include <opencv2/opencv.hpp>
#include "Image.h"
#include "dds/dds.h"

int main(int argc, char* argv[]) {
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  camera_Image msg;
  dds_qos_t* qos;
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  // Create a Participant.
  participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  // Create a Topic.
  topic = dds_create_topic(participant, &camera_Image_desc, "camera/Image",
                           NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  // Create a quality of service for the Reader and create the Reader.
  qos = dds_create_qos();
  // Reliable reader with a max blocking time of 1 second
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_MSECS(0));
  // Transient local durability
  dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
  // Keep all history to ensure we receive the latest image even if we start the subscriber after the publisher
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
  writer = dds_create_writer(participant, topic, qos, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));
  dds_delete_qos(qos);

  // Wait for a subscriber to match before writing data
  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));
  while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
    rc = dds_get_status_changes(writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));
    /* Polling sleep. */
    dds_sleepfor(DDS_MSECS(20));
  }

#ifdef _WIN32
  cv::VideoCapture cap(0, cv::CAP_DSHOW);
#elif __linux__
  cv::VideoCapture cap(0);
#endif
  //Force MJPG
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  //Set resolution
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  //Set FPS
  cap.set(cv::CAP_PROP_FPS, 30);
  //Reduce buffering
  cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

  size_t size{};
  {  // Grab one frame to get the size of the buffer needed for the DDS message
    cv::Mat frame;
    if (cap.read(frame)) {
      size = frame.total() * frame.elemSize();
      msg.data._length = size;
      msg.data._maximum = size;
      msg.data._release = false;
      msg.data._buffer = (uint8_t*)malloc(size);
    }
  }

  while (true) {
    cv::Mat frame;
    if (!cap.read(frame))
      continue;

    memcpy(msg.data._buffer, frame.data, size);
    msg.width = frame.cols;
    msg.height = frame.rows;
    msg.timestamp = std::chrono::steady_clock::now().time_since_epoch().count();

    dds_write(writer, &msg);
  }
  dds_free(msg.data._buffer);
  dds_delete(participant);
}
