#include "kinematics.h"
#include <Eigen/SVD>
#include <algorithm>
#include <queue>
#include <stack>
#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // Same as HW2, but have some minor change
  // Hint:
  //   1. If you don't use `axis` in this function, you can copy-paste your code
  // Note:
  //   1. bone.axis becomes quaternion instead of vector3f
  bone->startPosition = Eigen::Vector3f::Zero();
  bone->endPosition = Eigen::Vector3f::Zero();
  bone->rotation = Eigen::Quaternionf::Identity();

  auto R = posture.rotations;
  auto T = posture.translations;

  bone->startPosition += T[bone->idx];
  bone->endPosition = bone->startPosition + bone->direction * bone->length;
  bone->rotation = R[bone->idx];
  // traverse
  std::queue<Bone*> q;
  if (bone->child != NULL) {
    q.push(bone->child);
    bone = bone->child;
    while (bone->sibling != NULL) {
      q.push(bone->sibling);
      bone = bone->sibling;
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

Eigen::VectorXf leastSquareSolver(const Eigen::Matrix3Xf& jacobian, const Eigen::Vector3f& target) {
  // TODO (find x which min(| jacobian * x - target |))
  // Hint:
  //   1. Linear algebra - least squares solution
  //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
  // Note:
  //   1. SVD or other pseudo-inverse method is useful
  //   2. Some of them have some limitation, if you use that method you should check it.
   Eigen::VectorXf solution(jacobian.cols());
  solution.setZero();
  //svd 
  /* auto svd = jacobian.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& singularVal = svd.singularValues();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(jacobian.cols(), jacobian.rows());
  singularValuesInv.setZero();
  //singularValuesInv = singularVal.inverse();
  double pinvtoler = 1.e-6;
  for (int i = 0; i < singularVal.size(); i++) {
    if (singularVal(i) > pinvtoler) {
      singularValuesInv(i, i) = 1.0f / singularVal(i);
    } else {
      singularValuesInv(i, i) = 0.0f;
    }
  }
  //pseudo-inverse
  auto pJacobian = svd.matrixV() * singularValuesInv * svd.matrixU();
  solution = pJacobian * target; */
  solution = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target);
  return solution;
}

void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture) {
  constexpr int maxIterations = 10000;
  constexpr float epsilon = 1E-3f;
  constexpr float step = 0.1f;
  // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
  Bone* root = start - start->idx;
  std::vector<Bone*> boneList;
  // TODO
  // Hint:
  //   1. Traverse from end to start is easier than start to end (since there is only 1 parent)
  //   2. If start bone is not reachable from end. Go to root first.
  // Note:
  //   1. Both start and end should be in the list

  // Write your code here.
  bool reachable = true;
  Bone* cur = end;
  while (true) {
    if (cur->idx == start->idx) {
      boneList.insert(boneList.begin() ,cur);
      break;
    } else if (cur->idx == root->idx) {
      boneList.insert(boneList.begin(), cur);
      reachable = false;
      break;
    } else {
      boneList.insert(boneList.begin(), cur);
      cur = cur->parent;
    }
  }
  //from start to root
  if (!reachable) {
    std::stack<Bone*> q;
    cur = start;
      while (true) {
      if (cur->idx == root->idx) {
          break;
      } else {
        q.push(cur);
        cur = cur->parent;
      }
    }
      while (!q.empty()) {
        boneList.insert(boneList.begin(), q.top());
        q.pop();
      }
  }

  size_t boneNum = boneList.size();
  Eigen::Matrix3Xf jacobian(3, 3 * boneNum);
  jacobian.setZero();

  for (int i = 0; i < maxIterations; ++i) {
    forwardKinematics(posture, root);
    // TODO (compute jacobian)
    //   1. Compute jacobian columns
    //   2. Compute dTheta
    // Hint:
    //   1. You should not put rotation in jacobian if it doesn't have that DoF.
    //   2. jacobian.col(/* some column index */) = /* jacobian column */
    //   3. Call leastSquareSolver to compute dTheta

    // Write your code here.
     for (int i = 0; i < boneNum; i++) {
      Eigen::Vector3f deltaPos = boneList[i]->endPosition - boneList[i]->startPosition;
      if (boneList[i]->dofrx) {
        auto ai = boneList[i]->rotation.toRotationMatrix().col(0);
        jacobian.col(3 * i) = ai.normalized().cross(deltaPos);
      }
      if (boneList[i]->dofry) {
        auto ai = boneList[i]->rotation.toRotationMatrix().col(1);
        jacobian.col(1 + 3 * i) = ai.normalized().cross(deltaPos);
      }
      if (boneList[i]->dofrz) {
        auto ai = boneList[i]->rotation.toRotationMatrix().col(2);
        jacobian.col(2 + 3 * i) = ai.normalized().cross(deltaPos);
      }
    }

    auto dTheta = leastSquareSolver(jacobian, target - end->endPosition);

    for (size_t j = 0; j < boneNum; j++) {
      const auto& bone = *boneList[j];
      // TODO (update rotation)
      //   1. Update posture's eulerAngle using deltaTheta
      // Hint:
      //   1. Use posture.eulerAngle to get posture's eulerAngle
      //   2. All angles are in radians.
      //   3. You can ignore rotation limit of the bone.
      // Bonus:
      //   1. You cannot ignore rotation limit of the bone.

      // Write your code here.
      posture.eulerAngle[bone.idx] += Eigen::Vector3f(dTheta[0+j*3],dTheta[1+j*3],dTheta[2+j*3]) * step;
     // posture.eulerAngle[bone.idx][1] += dTheta[1] * step;
     // posture.eulerAngle[bone.idx][2] += dTheta[2] * step;

      posture.rotations[bone.idx] = Eigen::AngleAxisf(posture.eulerAngle[bone.idx][2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][0], Eigen::Vector3f::UnitX());

    }
  }
}
