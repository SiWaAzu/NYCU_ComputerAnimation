#include "kinematics.h"
#include <Eigen/Geometry>
#include <queue>
#include <algorithm>
#include <iostream>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton

  //initialzation
  bone->startPosition = Eigen::Vector3f::Zero();
  bone->endPosition = Eigen::Vector3f::Zero();
  bone->rotation = Eigen::Quaternionf::Identity();

  auto R = posture.rotations;
  auto T = posture.translations;

  bone->startPosition += T[bone->idx];
  bone->endPosition = bone->startPosition + bone->direction * bone->length;
  bone->rotation = R[bone->idx];
  //traverse
  std::queue<Bone*> q;
  if (bone->child != NULL) {
    q.push(bone->child);
    bone = bone->child;
    while (bone->sibling != NULL) {
      q.push(bone->sibling);
      bone = bone -> sibling;
    }
  }

  while (!q.empty()) {
    Bone* curr = q.front();

    curr->startPosition = Eigen::Vector3f::Zero();
    curr->endPosition = Eigen::Vector3f::Zero();
    curr->rotation = Eigen::Quaternionf::Identity();

    Eigen::Quaternionf Rm = Eigen::Quaternionf::Identity();

    Rm = (curr->rotationParentCurrent) * R[curr->idx];
    curr->rotation = curr->parent->rotation * Rm;
    curr->startPosition = curr->parent->endPosition;
    curr->endPosition = curr->startPosition + curr->rotation * (curr->direction * curr->length);

    if (curr->child != NULL) {
      q.push(curr->child);
      curr = curr->child;
      while (curr->sibling != NULL) {
        q.push(curr->sibling);
        curr = curr->sibling;
      }
    }
    q.pop();
  }

}

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());

  std::cout << "total:" << totalFrames;
  
  for (int i = 0; i < totalFrames; ++i) {
    // Maybe set some per=Frame variables here
    for (int j = 0; j < totalBones; ++j) {
      // TODO (Time warping)
      // original: |--------------|---------------|
      // new     : |------------------|-----------|
      // OR
      // original: |--------------|---------------|
      // new     : |----------|-------------------|
      // You should set these variables:
      //     newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      //     newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      // The sample above just set to initial state
      // Hint:
      //   1. Your should scale the frames before and after key frames.
      //   2. You can use linear interpolation with translations.
      //   3. You should use spherical linear interpolation for rotations.

      newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      double key, key1, key2;
      if (i <= newKeyframe)
        key = ((double)oldKeyframe) * i / (double)newKeyframe;
      else
        key = ((double)totalFrames - 1 - (double)oldKeyframe) * (i - newKeyframe) / (totalFrames - 1 - newKeyframe) +
              oldKeyframe;

      if (key == (int)key) {
        newMotion.posture(i).translations[j] = motion.posture(key).translations[j];
        newMotion.posture(i).rotations[j] = motion.posture(key).rotations[j];
        continue;
      }
      //interpolate
      key1 = (int)key;
      key2 = key1 + 1;
      if (key2 >= totalFrames) key2 = key1;
      //translation
      double a = key - key1;
      double b = key2 - key;
      newMotion.posture(i).translations[j] =
          (b * motion.posture(key1).translations[j] + a * motion.posture(key2).translations[j]) / (a + b);
      //rotation
      Eigen::Quaternionf newQ= motion.posture(key1).rotations[j].slerp(a / (a + b), motion.posture(key2).rotations[j]);
      newMotion.posture(i).rotations[j] = newQ;
    }
  }
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  int totalFrames = motionA.size();
  int totalB = motionB.size();
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.

  // Write your code here
  int minIdxA = 30;
  int minIdxB[matchRange] = {};
  
  
  /* for (int i = 0; i < matchRange; i++) {
   
    float mincost = 800000;
    for (int k = 0; k < totalB; k++) {
      float tmp = 0;
      for (int j = 0; j < motionA.posture(i).translations.size(); j++) {
        tmp += abs((motionB.posture(k).translations[j] - motionA.posture(i + totalFrames - 30).translations[j]).norm());
      }
    //std::cout << tmp << "\n";
      if (tmp <= mincost) {
        
        std::cout << k << "\n";
        mincost = tmp;
        minIdxA = i + totalFrames - 30;
        minIdxB[i] = k;
        difference[i] = mincost;
      }
      //std::cout << "Min:" << minIdxB << "\n";
    }
    /* float tmp = 0;
    for (int j = 0; j < motionA.posture(i).translations.size(); j++) {
      tmp += abs((motionB.posture(0).translations[j] - motionA.posture(i+totalFrames-30).translations[j]).sum());
    }
    difference[i] = tmp;
    if (tmp <= mincost) {
      mincost = tmp;
      minIdx = i + totalFrames - 30;
    } */
 /* }
  float min = 80000;
  int minIdx;
  for (int i = 0; i < matchRange; i++) {
    if (difference[i] < min) {
      minIdx = i;
    }
  }
  for (int i = 0; i <= minIdxA; i++) {
    newMotion.posture().push_back(motionA.posture(i));
  }
  //noblend test
  for (int i = minIdxB; i < totalB; i++) {
    newMotion.posture().push_back(motionB.posture(i));
  }
  //blending
  for (int i = 1; i <= blendFrameCount; i++) {
    
  }*/
  
  return newMotion;
}
