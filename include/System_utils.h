
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>

//read the file based on the txt path and pass to the Eigen Matrix;
//size is the length for each row/human
bool read_number_txt(const std::string txt_file_name, Eigen::MatrixXd& read_number_mat, int size = 10) //yuh read into Eigen Matrixd read_number_mat
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout<<"ERROR!!! Cannot read txt file "<<txt_file_name<<std::endl;
      	return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter=0;
    std::string line;
    if (read_number_mat.rows()==0)
  	read_number_mat.resize(100, size);
    //Check if the file is empty;
    if (filetxt.eof()) {
      std::cerr << "eof"<< std::endl;
      return false;
    }
    while (getline(filetxt,line))
    {
    	double t;
    	if(!line.empty()){
    	    std::stringstream ss(line);
    	    int colu=0;
    	    while (ss >> t)
    	    {
    	      read_number_mat(row_counter,colu)=t;
    	      colu++;
    	    }
    	    row_counter++;
    	    if (row_counter>=read_number_mat.rows()) // if matrix row is not enough, make more space.
    		read_number_mat.conservativeResize(read_number_mat.rows()*2,read_number_mat.cols());
    	     }
    }
    filetxt.close();

    read_number_mat.conservativeResize(row_counter,read_number_mat.cols());  // cut into actual rows

    return true;
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}