#include "utils.hpp"

#ifndef SEMANTIC_KITTI_LOADER_HPP
#define SEMANTIC_KITTI_LOADER_HPP

class KittiLoader {
public:
    KittiLoader(const std::string &abs_path) {
        pc_path_ = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "\033[1;31mError: The # of point clouds and # of labels are not same\033[0m" << std::endl;
        }
        std::cout << "Total " << num_frames_ << " files are loaded" << std::endl;
    }

    ~KittiLoader() {}

    size_t size() const { return num_frames_; }

    template <typename T>
    int get_quadrant(T &point)const
    {
        int res = 0;
        float x = point.x;
        float y = point.y;
        if (x > 0 && y >= 0)
            res = 1;
        else if (x <= 0 && y > 0)
            res = 2;
        else if (x < 0 && y <= 0)
            res = 3;
        else if (x >= 0 && y < 0)
            res = 4;
        return res;
    }

    template<typename T>
    void get_cloud(size_t idx, pcl::PointCloud<T> &non_ground_pc, pcl::PointCloud<T> &ground_pc, pcl::PointCloud<T> &raw_pc) const {
        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            throw invalid_argument("Could not open the .bin file!");
        }
        std::vector<float> buffer(1000000);
        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        // cloud.points.resize(num_points);

        if (std::is_same<T, pcl::PointXYZ>::value) {
            non_ground_pc.points.resize(num_points);
            for (int i = 0; i < num_points; i++) {
                auto &pt = non_ground_pc.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            non_ground_pc.points.resize(num_points);
            for (int i = 0; i < num_points; i++) {
                auto &pt = non_ground_pc.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
            }
        } else if (std::is_same<T, PointXYZILID>::value) {
            std::string label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
            std::ifstream label_input(label_name, std::ios::binary);
            if (!label_input.is_open()) {
                throw invalid_argument("Could not open the label!");
            }
            label_input.seekg(0, std::ios::beg);
            std::vector<uint32_t> labels(num_points);
            label_input.read((char*)&labels[0], num_points * sizeof(uint32_t));

            PointXYZILID pt;
            int previous_quadrant = 0;
            uint16_t ring_ = (uint16_t)64 - 1;

            for (int i = 0; i < num_points; i++) {
                int pt_label = labels[i] & 0xFFFF;

                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
                pt.label = labels[i] & 0xFFFF;
                pt.id = labels[i] >> 16;

                int quadrant = get_quadrant(pt);
                if (quadrant == 1 && previous_quadrant == 4 && ring_ > 0) {
                    ring_ -= 1;
                }
                pt.ring = ring_;
                raw_pc.push_back(pt);
                previous_quadrant = quadrant;

                //   1: "outlier"
                if (pt_label == 1){
                    continue;
                }
                //   40: "road"
                if (pt_label == 40){
                    ground_pc.push_back(pt);
                    continue;
                }
                //  44: "parking"
                if (pt_label == 44){
                    ground_pc.push_back(pt);
                    continue;
                }
                //   48: "sidewalk"
                if (pt_label == 48){
                    ground_pc.push_back(pt);
                    continue;
                }
                //   49: "other-ground"
                if (pt_label == 49){
                    ground_pc.push_back(pt);
                    continue;
                }
                //   60: "lane-marking"
                if (pt_label == 60){
                    ground_pc.push_back(pt);
                    continue;
                }
                // 72: "terrain"
                if (pt_label == 72){
                    ground_pc.push_back(pt);
                    continue;
                }

                non_ground_pc.points.push_back(pt);
            }
        }
    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
};

#endif //SEMANTIC_KITTI_LOADER_HPP