function outFileName = generateOutFileName(fileName)
time = clock+"";
timeStr = strcat(time(1),time(2),time(3),"_",time(4),"-",time(5));
outFileName = erase(fileName,".mat") + " " + timeStr + ".mat";
end

