#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

template <typename T>
class CSV
{
public:
    void write(const std::vector<std::vector<T>>& mat, const std::string& file_path);
    std::vector<std::vector<T>> read(const std::string& file_path);
    void read(std::vector<std::vector<T>>& mat, const std::string& file_path);
};

template <typename T>
void CSV<T>::write(const std::vector<std::vector<T>>& mat, const std::string& file_path)
{
    std::cout << "saving Q mat" << std::endl;

    std::ofstream file(file_path);
    for(const auto& row : mat){
	file << row[0];
    	for(int cell = 1; cell < row.size(); cell++)
	    file << "," << row[cell];
	file << std::endl;
    }

    file.close();
}

template <typename T>
std::vector<std::vector<T>> CSV<T>::read(const std::string& file_path)
{
    std::vector<std::vector<T>> parsed_data;
    std::ifstream file(file_path);

    std::string line;
    while(std::getline(file,line))
    {
        std::stringstream lineStream(line);
	std::string cell;
	std::vector<T> row;
        while(std::getline(lineStream,cell,','))
        {
            row.push_back(atoi(cell.c_str()));
        }
        parsed_data.push_back(row);
    }
    file.close();
    return parsed_data;
 }

template <typename T>
void CSV<T>::read(std::vector<std::vector<T>>& mat, const std::string& file_path)
{
    std::ifstream file(file_path);

    std::string line;
    int i = 0;
    while(std::getline(file,line))
    {
        std::stringstream lineStream(line);
	std::string cell;
	int j = 0;
        while(std::getline(lineStream,cell,','))
        {
            mat[i][j] = atoi(cell.c_str());
            j++;
        }
        i++;
    }
    file.close();
 }
