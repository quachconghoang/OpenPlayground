#include "DataIO.h"

int readSyncFileHeader(std::string fileName, std::vector<SyncFrame> & frame)
{
	std::ifstream headerFile;	headerFile.open(fileName.c_str());
	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }
	std::string temp;

	int line = 1;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;

	std::cout << std::fixed;
	while (!headerFile.eof())	{
		SyncFrame tHeader;
		headerFile >> temp;
		headerFile >> tHeader.depthImg;
		headerFile >> temp;
		headerFile >> tHeader.rgbImg;

		/*std::cout << "-Depth: " << tHeader.depthHeader.filePath
			<< " Color: " << tHeader.rgbHeaders.filePath
			<< " Time: " << tHeader.depthHeader.timeStamp << std::endl;*/
		if (tHeader.depthImg.size() > 5) frame.push_back(tHeader);
	}
	return 0;
}