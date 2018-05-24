/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* Modified for use within the OpenDLV framework by Marcus Andersson, Martin Baerveldt, Linus Eiderström Swahn and Pontus Pohl
* Copyright (C) 2018 Chalmers Revere
* For more information see <https://github.com/chalmers-revere/opendlv-perception-vision-orbslam2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "orbkeyframedatabase.hpp"

OrbKeyFrameDatabase::OrbKeyFrameDatabase (OrbVocabulary &voc):
    m_vocabulary(&voc)
{
    m_invertedFile.resize(voc.GetSize());
}


void OrbKeyFrameDatabase::Add(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lock(mMutex);

    for(OrbBowVector::const_iterator BowVector = keyFrame->m_bagOfWords.begin(), vend=keyFrame->m_bagOfWords.end(); BowVector!=vend; BowVector++)
        m_invertedFile[BowVector->first].push_back(keyFrame);
}

void OrbKeyFrameDatabase::Erase(std::shared_ptr<OrbKeyFrame> keyFrame)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(OrbBowVector::const_iterator vit=keyFrame->m_bagOfWords.begin(), vend=keyFrame->m_bagOfWords.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        std::list<std::shared_ptr<OrbKeyFrame>> &lKFs =   m_invertedFile[vit->first];

        for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(keyFrame==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void OrbKeyFrameDatabase::Clear()
{
    m_invertedFile.clear();
    m_invertedFile.resize(m_vocabulary->GetSize());
}


std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrameDatabase::DetectLoopCandidates(std::shared_ptr<OrbKeyFrame> keyFrame, float minScore)
{
    std::set<std::shared_ptr<OrbKeyFrame>> connectedKeyFrames = keyFrame->GetConnectedKeyFrames();
    std::list<std::shared_ptr<OrbKeyFrame>> keyFramesSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        std::unique_lock<std::mutex> lock(mMutex);

        for(OrbBowVector::const_iterator vit=keyFrame->m_bagOfWords.begin(), vend=keyFrame->m_bagOfWords.end(); vit != vend; vit++)
        {
            std::list<std::shared_ptr<OrbKeyFrame>> &keyFrames =   m_invertedFile[vit->first];

            for (auto candidate : keyFrames)
            {
                if(candidate->m_loopQuery!=keyFrame->m_id)
                {
                    candidate->m_loopWords=0;
                    if(!connectedKeyFrames.count(candidate))
                    {
                        candidate->m_loopQuery=keyFrame->m_id;
                        keyFramesSharingWords.push_back(candidate);
                    }
                }
                candidate->m_loopWords++;
            }
        }
    }

    if(keyFramesSharingWords.empty())
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    }

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > scoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for (auto &keyFramesSharingWord : keyFramesSharingWords)
    {
        if(keyFramesSharingWord->m_loopWords>maxCommonWords)
        {
            maxCommonWords= keyFramesSharingWord->m_loopWords;
        }
    }

    int minCommonWords = static_cast<int>(maxCommonWords*0.8f);

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (auto keyFrameSharingWords : keyFramesSharingWords)
    {
        if(keyFrameSharingWords->m_loopWords>minCommonWords)
        {
            nscores++;

            float si = static_cast<float>(m_vocabulary->GetScore(keyFrame->m_bagOfWords,
                                                                 keyFrameSharingWords->m_bagOfWords));

            keyFrameSharingWords->mLoopScore = si;
            if(si>=minScore)
            {
                scoreAndMatch.push_back(make_pair(si,keyFrameSharingWords));
            }
        }
    }

    if(scoreAndMatch.empty())
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    }

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > accumulateScoreAndMatch;
    float bestAccumulatedScore = minScore;

    // Lets now accumulate score by covisibility
    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=scoreAndMatch.begin(), itend=scoreAndMatch.end(); it!=itend; it++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = it->second;
        std::vector<std::shared_ptr<OrbKeyFrame>> neighbours = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        std::shared_ptr<OrbKeyFrame> pBestKF = pKFi;
        for (auto neighbour : neighbours)
        {
            if(neighbour->m_loopQuery==keyFrame->m_id && neighbour->m_loopWords>minCommonWords)
            {
                accScore+=neighbour->mLoopScore;
                if(neighbour->mLoopScore>bestScore)
                {
                    pBestKF=neighbour;
                    bestScore = neighbour->mLoopScore;
                }
            }
        }

        accumulateScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccumulatedScore)
        {
            bestAccumulatedScore=accScore;
        }
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccumulatedScore;

    std::set<std::shared_ptr<OrbKeyFrame>> spAlreadyAddedKF;
    std::vector<std::shared_ptr<OrbKeyFrame>> vpLoopCandidates;
    vpLoopCandidates.reserve(accumulateScoreAndMatch.size());

    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=accumulateScoreAndMatch.begin(), itend=accumulateScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrameDatabase::DetectRelocalizationCandidates(std::shared_ptr<OrbFrame> F)
{
    std::list<std::shared_ptr<OrbKeyFrame>> keyFramesSharingWords;

    // Search all keyframes that share a word with current frame
    {
        std::unique_lock<std::mutex> lock(mMutex);

        for(OrbBowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            std::list<std::shared_ptr<OrbKeyFrame>> &lKFs =   m_invertedFile[vit->first];

            for (auto pKFi : lKFs)
            {
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    keyFramesSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(keyFramesSharingWords.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for (auto &keyFramesSharingWord : keyFramesSharingWords)
    {
        if(keyFramesSharingWord->mnRelocWords>maxCommonWords)
        {
            maxCommonWords= keyFramesSharingWord->mnRelocWords;
        }
    }

    int minCommonWords = static_cast<int>(maxCommonWords*0.8f);

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=keyFramesSharingWords.begin(), lend= keyFramesSharingWords.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = static_cast<float>(m_vocabulary->GetScore(F->mBowVec, pKFi->m_bagOfWords));
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
    {
        return std::vector<std::shared_ptr<OrbKeyFrame>>();
    }

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (auto &it : lScoreAndMatch)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = it.second;
        std::vector<std::shared_ptr<OrbKeyFrame>> neighbours = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it.first;
        float accScore = bestScore;
        std::shared_ptr<OrbKeyFrame> pBestKF = pKFi;
        for (auto neighbour : neighbours)
        {
            if(neighbour->mnRelocQuery!=F->mnId)
            {
                continue;
            }

            accScore+=neighbour->mRelocScore;
            if(neighbour->mRelocScore>bestScore)
            {
                pBestKF=neighbour;
                bestScore = neighbour->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    std::set<std::shared_ptr<OrbKeyFrame>> spAlreadyAddedKF;
    std::vector<std::shared_ptr<OrbKeyFrame>> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (auto &it : lAccScoreAndMatch)
    {
        const float &si = it.first;
        if(si>minScoreToRetain)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = it.second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}