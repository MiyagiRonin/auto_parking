
#include "reverse_verticle_parking.h"

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <absl/strings/str_split.h>
#include "absl/strings/numbers.h"

using namespace apollo::common::math;

int main() {

    Pose start;
    std::vector<LineSegment2d> boundaries;
    
    std::string file_name = "../problems/reverse_verticle_1.txt";
	std::ifstream inFile;
	inFile.open(file_name, std::ios::in);
	if (!inFile.is_open())
	{
        std::cout <<"Can't open "<<file_name<<std::endl;
		return 1;
	}
	std::string buff;
    bool first_line = true;
	while (getline(inFile, buff))
	{
        const std::vector<absl::string_view> data = absl::StrSplit(buff, " ");
        if (first_line) {
            double x, y, theta;
            absl::SimpleAtod(data[0], &x);
            absl::SimpleAtod(data[1], &y);
            absl::SimpleAtod(data[2], &theta);
            start = {Vec2d(x, y), theta, Vec2d::CreateUnitVec2d(theta)};
            first_line = false;
        }else {
            double x1, y1, x2, y2;
            absl::SimpleAtod(data[0], &x1);
            absl::SimpleAtod(data[1], &y1);
            absl::SimpleAtod(data[2], &x2);
            absl::SimpleAtod(data[3], &y2);
            boundaries.emplace_back(Vec2d(x1, y1), Vec2d(x2, y2));
        }
        
	}
	inFile.close();

    LineCirclePath result;
    std::ofstream outFile;  
    outFile.open("output_path.txt", std::ios::out);

    const auto start_t = std::clock();
    const bool success = RunReverseVerticleParking(boundaries, start, &result);
    const double time = (double)(std::clock() - start_t) / CLOCKS_PER_SEC;
    printf("Time consuming is %.2f ms.\n", time * 1000.0);


    if (success) {
        const auto path = ConvertPathToDiscretePoses(result, 0.1);
        for(const auto& pose : path) {
            outFile<<pose.pos.x()<<" "<<pose.pos.y()<<" "<<pose.theta<<std::endl;
        }
        printf("Success!\n");
    }else{
        printf("Fail!\n");
    }
    outFile.close();

    return 0;
}