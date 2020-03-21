tonesQ_mutex.lock();
tone = tonesQ.front();
tonesQ.pop();
tonesQ.push(tone); 
tonesQ_mutex.unlock();

duration = tone.back();
interval = ((int)duration - (int)'0') * 125;