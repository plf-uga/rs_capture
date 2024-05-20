#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <numeric>
#include <chrono>
#include <ctime>
#include <thread>
#include <string>
#include <fstream>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

const char kPathSeparator =
#ifdef _WIN32
'\\';
#else
'/';
#endif

/**
 * This function gets the current date time
 * @return current datetime in the format of "YYYYMMDD_HHMMSS"
 */

string get_current_datetime ()
{
	auto current_datetime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	char datetime_str[20];
	strftime(datetime_str, sizeof(datetime_str), "%Y%m%d_%H%M%S", std::localtime(&current_datetime));
	string current_datetime_str(datetime_str);
	return  datetime_str;
}




float box_dist(const rs2::depth_frame& depth_frame, int x_min, int x_max, int y_min, int y_max )
{
	//vector to stored the distances inside the region of interest
	vector<float> distance;
	
	for (int i = x_min; i < x_max; i++)
	{
	  for(int j = y_min; j < y_max; j++)
	  {
	    float depth_meters = depth_frame.get_distance(i, j);
            if (depth_meters >0)
	    {
    		distance.push_back(depth_meters);
	    }
  
	  }	
	}
	float mean_distance = 0.0f;
	mean_distance = std::accumulate(distance.begin(), distance.end(), 0.0f) / distance.size();
	return mean_distance;
}

void readParFile(const string& filename, vector<string*>& pointers)
{

	std::ifstream file(filename.c_str());
	string line;

	if(!file.is_open())
	{
		cerr << "Unable to open input file " << filename << endl;
		cout << "Warning!! All config parameters will be set as default" << endl;
	
	}else
	{
		int i = 0;
		while (getline(file, line))
		{
		 if (line.empty()||line[0] =='#')
		  {
		    continue;

		  }else
		  {
		    
		    string tmp = line;
		    *pointers[i] = tmp;
		    i++;
		   // cout << tmp <<endl;
		  }
		}
	}	
	try
	{
	file.close();	 	
	}
	catch(const exception& e) {
	  cerr << "Unable to close file " << endl;
	}
	
}

void write_log(std::string type, std::string message, int verbose, const std::string& path, const std:: string& cam_name)
{

	auto current_datetime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	char datetime_str[20];
	strftime(datetime_str, sizeof(datetime_str), "%Y%m%d %H:%M:%S", std::localtime(&current_datetime));
	std::string filePath = path + "/" + cam_name + "_log.txt";	

	std::stringstream full_message_text;
	full_message_text << type << ": " << datetime_str << " --- " << message;
	if (type == "ERROR")
		std::cerr << full_message_text.str() << std::endl;
	else
		std::cout << full_message_text.str() << std::endl;
	if ((verbose == 1 && type == "ERROR") || (verbose == 2))
	{
		std::ofstream outfile;
		outfile.open(filePath.c_str(), std::ios::out | std::ios_base::app);
		outfile << full_message_text.str() << std::endl;
		outfile.close();		
	}
}

/**
 * This creates a new directory based on a given name
 * @directoryName is the desired name for the new directory
 * @return new directory in the current working directory
 */

void create_dir(const string& directoryName, int verbose, string path, string cam_name)
{
	
	int status = mkdir(directoryName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (status == 0) {        
	write_log("INFO", directoryName+" created successfully.", verbose, path, cam_name);
    	} else {
        write_log("INFO", directoryName + " already exists!", verbose, path, cam_name);    	    	
	}
}

int main() {
	string fileName;
	cout << "Please type the parameter file path: " << endl;
	getline(cin, fileName);//= "/home/alveslab/rs_capture/input.par";
	//Getting the starting working directory where the log file will be saved
		
	char buffer[50];
	getcwd(buffer, sizeof(buffer));
	string startDirectory = std::string(buffer);
	string final_dir;
	std::ostringstream oss;		
	
	string msg;
	string target_directory, cam_name, W, H, fps, distance, x_min, x_max, y_min, y_max, rgbAE, laserIR, allowRGB, verbose1, dev_num, run_length;
	vector<string*> ptrs = {&target_directory, &cam_name, &W, &H, &fps, &distance, &x_min, &x_max, &y_min, &y_max, &rgbAE, &laserIR, &allowRGB, &verbose1, &dev_num, &run_length};
	
	readParFile (fileName, ptrs);
	int verbose = stoi(verbose1);
	int dev_choice = stoi(dev_num);
	int numAtpp = 4;
	int currentAtpp = 0;
	auto duration = std::chrono::minutes(stoi(run_length));
	
	vector <string> parNames = {"Target Directory: ", "Camera Name: ", "Image Width: ",
				"Image Height: ", "Frames per Second: ", "Distance from ROI: ",
 				"x_min: ", "x_max: ", "y_min: ", "y_max: ",
 				"Auto Exposure Priority for RGB (0 = No, 1 = Yes): ",
 				"Allow IR laser (0 = No, 1 = Yes): ",
 				"Allow RGB capture (0 = No, 1 = Yes): ", "verbose: ", "Device Number: ", "Time to run (minutes): "};
	
	if (chdir(target_directory.c_str())==0){
		
		final_dir = target_directory;		
		oss.str("");
		oss << 	"Working directory set to " << target_directory;
		write_log("INFO",oss.str() , verbose, final_dir, cam_name); 
	}else{
		final_dir = startDirectory;	
		oss.str("");
		write_log("ERROR", "Failed to set directory. Reason: directory does not exist!!", verbose, target_directory, cam_name);	
		oss << "Using " << buffer << " as working directory";
		write_log("WARNING", oss.str(), verbose, final_dir, cam_name);	
		target_directory = buffer;
	}
	
	
	if (run_length == "-1"){duration = std::chrono::minutes(10);}					

	write_log("INFO", "####---List of parameters configured ---###", verbose, final_dir, cam_name);	
	for (int i = 0; i < parNames.size(); i++)
	{
	  msg = parNames[i] + " " + *ptrs[i];
	  write_log("INFO", msg, verbose, final_dir, cam_name);
	}
	cout << endl;
	
	
	//Check if camera is connected
	rs2::context ctx;
	auto devices = ctx.query_devices();
	if (devices.size() == 0)
	{
		write_log("ERROR", "No realsense device connected, please check USB connectivity", verbose, final_dir, cam_name);
		return 1; 
	}
	else
	{
		std::ostringstream out;
		out << "Number of rs devices connected: " << devices.size();		
		write_log("INFO",  out.str(), verbose, final_dir, cam_name);
	}

	
	auto dev = devices[dev_choice]; //Chose which camera to use (In case of multiple cameras connected)	
	//Check if camera is streaming frames	
	while (currentAtpp < numAtpp)
	{
		rs2::pipeline piptest;
		rs2::config cfg_test;		
		cfg_test.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		piptest.start(cfg_test);
	
		try 
		{
			for (auto i = 0; i < 30; i++) piptest.wait_for_frames();
			//If sucess continue pipeline
			write_log("INFO", "Frame streaming estabilhed, ready to start...", verbose, final_dir, cam_name);
			piptest.stop(); //stop the current test pipeline					
			break;
		}catch(const rs2::error& e)
		{
			std::stringstream error_text;
			error_text << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what();
			write_log("ERROR", error_text.str(), verbose, final_dir, cam_name);
			cout << endl;
			write_log("INFO", "Trying to restart camera ... ", verbose, final_dir, cam_name);
			std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait before reattempting
			dev.hardware_reset();
			std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait before reattempting
			currentAtpp++;
			piptest.stop();
			if(currentAtpp==numAtpp)
			{
			piptest.stop();
			write_log("ERROR", "unable to get frames, please check USB connectivity", verbose, final_dir, cam_name);
			return 1;
			}
		}		
		
	}	
	

	rs2::colorizer color_map;
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));	
	if (allowRGB=="1")
	{
	cfg.enable_stream(RS2_STREAM_COLOR, stoi(W), stoi(H), RS2_FORMAT_RGB8, stoi(fps));
	}	
	cfg.enable_stream(RS2_STREAM_DEPTH, stoi(W), stoi(H), RS2_FORMAT_Z16, stoi(fps));
    	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	pipe.start(cfg);
	
	
	
	if (allowRGB=="1")
	{
	auto rgb_sensor = pipe.get_active_profile().get_device().query_sensors()[1];
	rgb_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, stoi(rgbAE));
	}

	auto depth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
	depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, stoi(laserIR));
	
	time_t start = time(nullptr);
	oss << "Camera triggered at: " << ctime(&start);	
    	write_log("INFO", oss.str(), verbose, final_dir, cam_name);	
	
	

	//Create a directory named by date to store the images		
	string date = get_current_datetime();
	date = date.substr(0, 8);
    	string directoryName = cam_name+"_"+date;	
	create_dir(directoryName, verbose, final_dir, cam_name);    	
	//
	string current_dir = final_dir+"/"+directoryName;
	write_log("INFO", "Images will be saved at: " + current_dir, verbose, final_dir, cam_name);	
	
	//Creating folders to store data	
	
	//Binary Depth Directory
	string depthdir = current_dir+"/depth_bin";
	create_dir(depthdir, verbose, final_dir, cam_name);
	
	//Color frames directory (if allowed)	
	string colordir = current_dir+"/rgb_png";
	if (allowRGB=="1")
	{
	create_dir(colordir, verbose, final_dir, cam_name);
	}

	//Colored depth directory
	string col_depthdir = current_dir+"/depth_png";
	create_dir(col_depthdir, verbose, final_dir, cam_name);

	//IR frames directory
	string irdir = current_dir+"/IR_png";
	create_dir(irdir, verbose, final_dir, cam_name);	
	
	auto begin = std::chrono::steady_clock::now();		
	int frame_count = 0;

	//BEGIN MAIN LOOP 
	while (std::chrono::steady_clock::now() - begin < duration)
	{		
	  if (run_length == "-1"){begin = chrono::steady_clock::now() + std::chrono::minutes(10);}//Runs the camera for indefinite time	
	  try{
		rs2::frameset frame = pipe.wait_for_frames();
	  	auto dframe = frame.get_depth_frame();
	  	//float dist = dframe.get_distance(dframe.get_width() / 2, dframe.get_height() / 2);
	 	 float dist = box_dist(dframe, stoi(x_min),stoi(x_max),stoi(y_min),stoi(y_max));  
	  	//cout << dist << " m" << endl;
	 	 	  	            
	 	 if (dist < stof(distance))
       		 {
			frame_count++; // Increment the frame count
			write_log("INFO", "Average distance from ROI is " + std::to_string(dist) + " m", verbose, final_dir, cam_name);  		
	    		string time_now = get_current_datetime();

	    		if (frame && frame.get_data())
	    		{
		 	 // Save depth frame as binary
	    	    
	        	string depthFilename = depthdir+"/"+time_now+"_"+to_string(frame_count)+"_d.bin";    
                  	ofstream(depthFilename, ios::binary).write((char*)dframe.get_data(), dframe.get_width() * dframe.get_height()*dframe.get_bytes_per_pixel());
			write_log("INFO", "Saved " + depthFilename, verbose, final_dir, cam_name);	
	       
		  	// Save the color image
			if (allowRGB=="1")
			{
	      	  	auto cframe = frame.get_color_frame();	    
	      	  	string colorFilename = colordir+"/"+time_now+"_"+"c"+".png";            
              	  	stbi_write_png(colorFilename.c_str(), cframe.get_width(), cframe.get_height(),cframe.get_bytes_per_pixel(), cframe.get_data(), cframe.get_stride_in_bytes());            
              	  	write_log("INFO", "Saved " + colorFilename, verbose, final_dir, cam_name);		
	      	 	}
		
	         	// Save IR image
	         	auto iframe = frame.get_infrared_frame();	    
	         	string iFilename = irdir+"/"+time_now+"_"+"i"+".png";            
                 	stbi_write_png(iFilename.c_str(), iframe.get_width(), iframe.get_height(),iframe.get_bytes_per_pixel(), iframe.get_data(), iframe.get_stride_in_bytes());
	         	write_log("INFO", "Saved " + iFilename, verbose, final_dir, cam_name);           
              	
	         	// Save the colored depth image as png
	         	// Colorize the depth frame
                 	rs2::video_frame d_color = color_map.colorize(dframe);	    
	         	string col_depthFilename = col_depthdir+"/depth_"+time_now+"_"+"d"+".png";
	         	stbi_write_png(col_depthFilename.c_str(), d_color.get_width(), d_color.get_height(), d_color.get_bytes_per_pixel(), d_color.get_data(),  d_color.get_stride_in_bytes());             
                 	write_log("INFO", "Saved " + col_depthFilename, verbose, final_dir, cam_name);       	    
            
              		
	      		}	    
	   	}
         }catch (const rs2::error& e)	{
		std::stringstream error_text;
		error_text << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what();
		//cout << error_text.str() << endl;
		write_log("ERROR", error_text.str(), verbose, final_dir, cam_name);
		write_log("INFO", "Trying to restart camera ... ", verbose, final_dir, cam_name);
		std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait before reattempting
		dev.hardware_reset();
		std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait before reattempting		
	}	
    
       }
	pipe.stop();
	
	std::stringstream final_msg;
	final_msg <<  "Exiting application with no errors.";
	write_log("INFO", final_msg.str() , verbose, final_dir, cam_name);
	
	return 0;
}

