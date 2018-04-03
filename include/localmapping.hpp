//
// Created by shiratori on 2018-04-03.
//

#ifndef OPENDLV_LOCALMAPPING_HPP
#define OPENDLV_LOCALMAPPING_HPP

class LocalMapping
{
public:
    void InsertKeyFrame(std::shared_ptr<OrbKeyFrame> pKF);
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void InterruptBA();
    int KeyframesInQueue();
    bool SetNotStop(bool flag);
    void RequestReset();

};

#endif //OPENDLV_LOCALMAPPING_HPP
