#include "FileManager.h"

FileManager::FileManager(const string &path, const string &fileName)
{
    this->path = path;
    this->fileName = fileName;
    csvFile.open(path + "/" + fileName);
    writeFile("id,timestamp, lat, long, mode, isCommunicating, direction, reached, partnerId, partnerMode, broadcastedPath, receivedPath, beforePathLength, afterPathLength, sentDataSize, receivedDataSize");
}

void FileManager::writeFile(const string &data)
{
    // cout<<data<<endl;
    csvFile << data << '\n';
    csvFile.flush();
}

void FileManager::closeFile()
{
    csvFile.close();
}