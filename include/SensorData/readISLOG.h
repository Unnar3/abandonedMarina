class ReadISLOG{    
public:
    struct isData{
        std::vector<double> logtime;
        std::vector<double> sensortime;
        std::vector<double> transducerangle;
        std::vector<std::vector<int> > bins;
    };


    // Read in the sonar data
    isData readISLOG(void){
        
        std::cout << "Started reading Sonar data!" << std::endl;

        std::ifstream file("/home/unnar/catkin_ws/src/abandonedmarina/_040825_1735_IS.log");
        std::string line; 
        // std::getline(file, line);
        int line_number = 0;
        std::cout.precision(13);
        // std::cout << line << std::endl;
        double number = 0;
        isData is_data;
        std::vector<int> single_bins(500);
        while (std::getline(file, line))
        {
            std::stringstream   linestream(line);
            std::string         data;

            if (line_number > 7){
                if (!line.empty() || line!=""){
                    // std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).
                    for(int i = 0; i < 503; i++){
                        linestream >> number;
                        if (i == 0){
                            is_data.logtime.push_back(number);
                        } else if (i == 1){
                            is_data.sensortime.push_back(number);
                        } else if (i == 2){
                            is_data.transducerangle.push_back(number);
                        } else {
                            single_bins.at(i-3) = (int)number;
                        }
                    }
                    is_data.bins.push_back(single_bins);
                }
            }
            line_number++;
        }
        std::cout << "Finished reading Sonar data!" << std::endl;
        return is_data;
    }

};