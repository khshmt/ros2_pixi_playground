#include <chrono>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <thread>
#include "Image.h"
#include "dds/dds.h"

#define MAX_SAMPLES 1

int main(int argc, char* argv[]) {
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t reader;
  camera_Image* msg;
  void* samples[MAX_SAMPLES];
  dds_sample_info_t infos[MAX_SAMPLES];
  dds_return_t rc;
  dds_qos_t* qos;
  (void)argc;
  (void)argv;

  // rerun setup
  const rerun::RecordingStream rec("DDS_camera_stream");
  rec.spawn(rerun::SpawnOptions{.memory_limit = "1GB"}).exit_on_failure();
  if (!rec.is_enabled()) {
    DDS_FATAL("Failed to start rerun viewer!!\n");
  }

  // Create a Participant.
  participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  // Create a Topic.
  topic = dds_create_topic(participant, &camera_Image_desc, "camera/Image",
                           NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  // Create a reliable Reader.
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  reader = dds_create_reader(participant, topic, qos, NULL);
  if (reader < 0)
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  samples[0] = camera_Image__alloc();
  while (true) {
    rc = dds_read(reader, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
    if (rc < 0)
      DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));
    if (rc > 0 && infos[0].valid_data) {
      msg = static_cast<camera_Image*>(samples[0]);
      rec.log("camera/image", rerun::Image(msg->data._buffer,
                                           {static_cast<uint32_t>(msg->width),
                                            static_cast<uint32_t>(msg->height)},
                                           rerun::datatypes::ColorModel::BGR));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  camera_Image_free(samples[0], DDS_FREE_ALL);
  // Deleting the participant will delete all its children recursively as well.
  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return 0;
}