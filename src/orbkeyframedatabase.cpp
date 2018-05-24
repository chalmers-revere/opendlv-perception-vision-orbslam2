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
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.getSize());
}


void OrbKeyFrameDatabase::add(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lock(mMutex);

    for(OrbBowVector::const_iterator vit= pKF->m_bagOfWords.begin(), vend=pKF->m_bagOfWords.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void OrbKeyFrameDatabase::erase(std::shared_ptr<OrbKeyFrame> pKF)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(OrbBowVector::const_iterator vit=pKF->m_bagOfWords.begin(), vend=pKF->m_bagOfWords.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        std::list<std::shared_ptr<OrbKeyFrame>> &lKFs =   mvInvertedFile[vit->first];

        for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void OrbKeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->getSize());
}


std::vector<std::shared_ptr<OrbKeyFrame>> OrbKeyFrameDatabase::DetectLoopCandidates(std::shared_ptr<OrbKeyFrame> pKF, float minScore)
{
    std::set<std::shared_ptr<OrbKeyFrame>> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    std::list<std::shared_ptr<OrbKeyFrame>> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        std::unique_lock<std::mutex> lock(mMutex);

        for(OrbBowVector::const_iterator vit=pKF->m_bagOfWords.begin(), vend=pKF->m_bagOfWords.end(); vit != vend; vit++)
        {
            std::list<std::shared_ptr<OrbKeyFrame>> &lKFs =   mvInvertedFile[vit->first];

            for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                std::shared_ptr<OrbKeyFrame> pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = static_cast<int>(maxCommonWords*0.8f);

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = static_cast<float>(mpVoc->getScore(pKF->m_bagOfWords,pKFi->m_bagOfWords));

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = it->second;
        std::vector<std::shared_ptr<OrbKeyFrame>> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        std::shared_ptr<OrbKeyFrame> pBestKF = pKFi;
        for(std::vector<std::shared_ptr<OrbKeyFrame>>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<OrbKeyFrame> pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    std::set<std::shared_ptr<OrbKeyFrame>> spAlreadyAddedKF;
    std::vector<std::shared_ptr<OrbKeyFrame>> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
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
    std::list<std::shared_ptr<OrbKeyFrame>> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        std::unique_lock<std::mutex> lock(mMutex);

        for(OrbBowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            std::list<std::shared_ptr<OrbKeyFrame>> &lKFs =   mvInvertedFile[vit->first];

            for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                std::shared_ptr<OrbKeyFrame> pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = static_cast<int>(maxCommonWords*0.8f);

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(std::list<std::shared_ptr<OrbKeyFrame>>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = static_cast<float>(mpVoc->getScore(F->mBowVec,pKFi->m_bagOfWords));
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return std::vector<std::shared_ptr<OrbKeyFrame>>();

    std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        std::shared_ptr<OrbKeyFrame> pKFi = it->second;
        std::vector<std::shared_ptr<OrbKeyFrame>> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        std::shared_ptr<OrbKeyFrame> pBestKF = pKFi;
        for(std::vector<std::shared_ptr<OrbKeyFrame>>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<OrbKeyFrame> pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
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
    for(std::list<std::pair<float,std::shared_ptr<OrbKeyFrame>> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            std::shared_ptr<OrbKeyFrame> pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}