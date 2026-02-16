#include "dds/dds.h"
#include "Image.h"
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    dds_entity_t participant;
    dds_entity_t topic;
    dds_entity_t writer;
    dds_return_t rc;
    camera_Image msg;
    uint32_t status = 0;
    (void)argc;
    (void)argv;

    participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0)
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

    topic = dds_create_topic(
        participant,
        &camera_Image_desc,
        "camera/Image",
        NULL,
        NULL);
    if (topic < 0)
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

    writer = dds_create_writer(participant, topic, NULL, NULL);
    if (writer < 0)
        DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

    rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
    if (rc != DDS_RETCODE_OK)
        DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

    while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
    {
        rc = dds_get_status_changes(writer, &status);
        if (rc != DDS_RETCODE_OK)
            DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

        /* Polling sleep. */
        dds_sleepfor(DDS_MSECS(20));
    }
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FPS, 30);

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame))
            continue;

        size_t size = frame.total() * frame.elemSize();
        msg.data._length = size;
        msg.data._maximum = size;
        msg.data._release = true;
        msg.data._buffer = (uint8_t*)malloc(size);
        memcpy(msg.data._buffer, frame.data, size);
        msg.width = frame.cols;
        msg.height = frame.rows;
        msg.timestamp = std::chrono::steady_clock::now().time_since_epoch().count();

        dds_write(writer, &msg);
    }
    dds_delete(participant);
}
