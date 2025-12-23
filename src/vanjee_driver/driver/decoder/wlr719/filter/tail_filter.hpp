#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

namespace vanjee {
namespace lidar {
// h
#define VERSION "1.1.0"
inline double ANGLE2RADIAN(double x) {
  return (x * M_PI / 180.0);
}

#define MaxLEN 10800
int LEN = MaxLEN;
float ANGLE_RESO = 0.00116f;
bool bNeedInit = true;
typedef float rangeType;

enum TAILFLAG { forceNT = -1, mayNT = 0, isT = 1 };

typedef struct s_Config_ {
  int s_iSampleNum;
  rangeType s_fSampleDistence;
  rangeType s_fRangeError;
  float s_fRangeMulti;
  rangeType s_fRangeMin;
  rangeType s_fRangeMax;
  float s_fTailAngle;
  rangeType s_fSaveRangeDiff;
  rangeType s_rRangeSave;
  int s_iSaveMinNum;
  int s_iTailMinNum;
  int s_iTailMaxNum;
  bool FTail;
  rangeType s_fErrorCode;
  rangeType s_outlierRngDiff;
  int noTailMinSize;
  rangeType s_fSampleMinDist;
  float s_fSampleMulti;
  rangeType s_fFTailRangeError;
  float s_fSmallestSize;
  float s_fSmallestMulti;
} s_Config;

typedef struct s_Sample_ {
  int s_iCnt;
  rangeType s_fRngs[MaxLEN];
  int s_iIds[MaxLEN];
  char s_bTailsCoeffs[MaxLEN];
  bool s_bTails[MaxLEN];
} s_Sample;

float *generateTailLUT(float minAngle);
void tfinit();
#ifdef _WIN32
#pragma region Sample
#endif
void deleteSampleData(s_Sample *spl, int id);
void addSampleData(rangeType range, int id, s_Sample *sampleData);
bool isRangeOutlier(rangeType rangeCurr, rangeType rangeLast, float coeff);
TAILFLAG isTail(float tailCoeff, int interAngle, float *tailLUT, rangeType range);
void reset();
#ifdef _WIN32
#pragma endregion Sample
#endif

#ifdef _WIN32
#pragma region Segment
#endif
typedef struct OneSec_ {
  int s_iSplSta;
  int s_iSplEnd;
  int s_iRawSta;
  int s_iRawEnd;
  int s_iPCnt;
} OneSec;

typedef struct Sections_ {
  OneSec s_secs[MaxLEN];
  int s_iCnt;
} Sections;

bool checkTail(s_Sample *spl, int id, int window);
void buildOneSec(OneSec *oneSec, s_Sample *spl, int sta, int end);
bool addSection(Sections *secs, s_Sample *spl, int sta, int end);
#ifdef _WIN32
#pragma endregion Segment
#endif

#ifdef _WIN32
#pragma region AnaSection
#endif
void anaSection(Sections *sec, int id, s_Sample *spl, rangeType *ranges);
void findOutlier(rangeType *ranges, int stdId, int startId, int endId, int *outId_F, int *outId_B);
void isSectionTail(rangeType *ranges, int sta, int end);
#ifdef _WIN32
#pragma endregion AnaSection
#endif
extern float *g_tailLUT;
extern s_Config g_sysConfig;
extern s_Sample g_splData;
extern Sections g_sec;
extern int printLevel;

void sample(rangeType *ranges, s_Sample *sampleData);
void segmentData(s_Sample *spl, rangeType *ranges, Sections *sec);
void filter(Sections *sec, s_Sample *spl, rangeType *ranges);
void printSection(OneSec *sec, int idx);
void findFBGround(OneSec *sec, s_Sample *spl, rangeType *ranges, int *sta, int *end);
int isFB(rangeType *ranges, int id);
void filterFTail(rangeType *ranges);
void process(rangeType *ranges, int hz);
// rangeType calcMiddle(rangeType r1, int idDiff1, rangeType r2, int idDiff2);
// rangeType calcNext(rangeType r1, int idDiff1, rangeType r2, int idDiff2);
// bool checkLiner(s_Sample *spl, int id, float *Coeff);
bool isMerge(s_Sample *spl, int newSecSta, OneSec *lastSec, int window);
void mergeSec(s_Sample *spl, Sections *secs, int newSta, int newEnd);
void buildSection(Sections *secs, s_Sample *spl, int newSecSta, int newSecEnd, int mergeWindow);
void findSectionSE(OneSec *sec, s_Sample *spl, rangeType *ranges, int *tailSecRawSta, int *tailSecRawEnd);
void deleteSection(Sections *secs, int id);

// cpp
float *g_tailLUT;
float *g_noTailLUT;
s_Config g_sysConfig;
s_Sample g_splData;
Sections g_sec;
bool tailId[MaxLEN];
int printLevel = 0;
float FTAIL_LUT[2] = {0.99, 1.01};

void tfinit() {
  g_sysConfig.s_iSampleNum = 20;
  g_sysConfig.s_fSampleDistence = 0.03;
  g_sysConfig.s_fSampleMinDist = 0.03;
  g_sysConfig.s_fSampleMulti = 0.02;
  g_sysConfig.noTailMinSize = 5;
  g_sysConfig.s_fRangeError = 0.005;
  g_sysConfig.s_fRangeMulti = 0.01;
  g_sysConfig.s_fRangeMin = 0.1;
  g_sysConfig.s_fErrorCode = 0.02;
  g_sysConfig.s_fRangeMax = 10;
  g_sysConfig.s_fTailAngle = 15;
  g_sysConfig.s_fSaveRangeDiff = 0.02;
  g_sysConfig.s_iSaveMinNum = 4;
  g_sysConfig.s_iTailMinNum = 1;
  g_sysConfig.s_iTailMaxNum = 200;
  g_sysConfig.FTail = true;
  g_sysConfig.s_fFTailRangeError = 0.002;
  g_sysConfig.s_outlierRngDiff = 0.5;
  g_sysConfig.s_fSmallestSize = 0.01;
  g_sysConfig.s_fSmallestMulti = 60;
  g_tailLUT = generateTailLUT((g_sysConfig.s_fTailAngle));
  g_noTailLUT = generateTailLUT((40));

  float FTailAngle = ANGLE2RADIAN(20);
  FTAIL_LUT[0] = sin(FTailAngle - ANGLE_RESO) / sin(M_PI - FTailAngle);
  FTAIL_LUT[1] = sin(M_PI - FTailAngle - ANGLE_RESO) / sin(FTailAngle);
}

void reset() {
  memset(tailId, false, MaxLEN * sizeof(bool));
  g_splData.s_iCnt = 0;
  g_sec.s_iCnt = 0;
}

float *generateTailLUT(float minAngle) {
  float *tailLUT = new float[60];
  float minRadian = ANGLE2RADIAN(minAngle);
  float radian = 0;
  float maxRadian = 0;
  float minMulti = 0, maxMulti = 0;
  for (int i = 0; i < 30; ++i) {
    radian = ANGLE_RESO * i;
    maxRadian = M_PI - minRadian;
    minMulti = sin(minRadian - radian) / sin(maxRadian);
    maxMulti = sin(maxRadian - radian) / sin(minRadian);
    tailLUT[i * 2] = minMulti;
    tailLUT[i * 2 + 1] = maxMulti;
  }
  return tailLUT;
}

#ifdef _WIN32
#pragma region Sample
#endif

void sample(rangeType *ranges, s_Sample *spl) {
  reset();
  bool inited = false;
  rangeType lastNanRange = 0;
  int lastNanIdx = -1;
  // rangeType rangeError = g_sysConfig.s_fRangeError;
  int splLastId = 0;
  int constNanCnt = 0;
  for (int i = 0; i < LEN; ++i) {
    if (ranges[i] < g_sysConfig.s_fErrorCode) {
      constNanCnt++;
      continue;
    }

    if (inited) {
      splLastId = spl->s_iIds[spl->s_iCnt - 1];
      if (constNanCnt >= 3) {
        if (lastNanIdx != spl->s_iIds[splLastId]) {
          addSampleData(lastNanRange, lastNanIdx, spl);
        }
        addSampleData(ranges[i], i, spl);
        spl->s_bTailsCoeffs[spl->s_iCnt - 1] = TAILFLAG::isT;
      } else if (isRangeOutlier(ranges[i], ranges[splLastId], g_sysConfig.s_fSampleMulti)) {
        if ((fabs(ranges[i] - lastNanRange) >= g_sysConfig.s_fSampleMinDist) && lastNanIdx > splLastId) {
          if (((fabs(lastNanRange - ranges[splLastId]) >= g_sysConfig.s_fSampleMinDist) || lastNanIdx - splLastId >= 2)) {
            addSampleData(lastNanRange, lastNanIdx, spl);
          } else {
            deleteSampleData(spl, -1);
            addSampleData(lastNanRange, lastNanIdx, spl);
          }
        }
        addSampleData(ranges[i], i, spl);
      } else if (i - splLastId >= g_sysConfig.s_iSampleNum) {
        addSampleData(ranges[i], i, spl);
      }
    } else {
      addSampleData(ranges[i], i, spl);
      inited = true;
    }
    constNanCnt = 0;
    lastNanIdx = i;
    lastNanRange = ranges[i];
  }
}

void deleteSampleData(s_Sample *spl, int id) {
  if (id == -1) {
    if (spl->s_iCnt > 0)
      spl->s_iCnt -= 1;
    else
      spl->s_iCnt = 0;
  } else if (id < spl->s_iCnt) {
    // todo
  }
}

void addSampleData(rangeType range, int id, s_Sample *spl) {
  int offset = spl->s_iCnt;
  spl->s_fRngs[offset] = range;
  spl->s_iIds[offset] = id;
  if (0 == offset) {
    spl->s_bTailsCoeffs[offset] = TAILFLAG::mayNT;
  } else if ((range < g_sysConfig.s_fRangeMin || range > g_sysConfig.s_fRangeMax)) {
    spl->s_bTailsCoeffs[offset] = TAILFLAG::forceNT;
  } else {
    rangeType diff = range - spl->s_fRngs[offset - 1];
    rangeType thr = range < spl->s_fRngs[offset - 1] ? range * g_sysConfig.s_fRangeMulti : spl->s_fRngs[offset - 1] * g_sysConfig.s_fRangeMulti;
    if (thr < g_sysConfig.s_fRangeError)
      thr = g_sysConfig.s_fRangeError;
    float coeff = range / spl->s_fRngs[offset - 1];
    if ((diff > thr || diff < -thr)) {
      spl->s_bTailsCoeffs[offset] = isTail(coeff, id - spl->s_iIds[offset - 1], g_tailLUT, range);
    } else {
      spl->s_bTailsCoeffs[offset] = TAILFLAG::mayNT;
    }
    if ((id - spl->s_iIds[offset - 1] > 5) || (diff > 0.03 || diff < -0.03)) {
      TAILFLAG tailFlag = isTail(coeff, id - spl->s_iIds[offset - 1], g_noTailLUT, range);
      if (tailFlag != TAILFLAG::isT)
        spl->s_bTailsCoeffs[offset] = TAILFLAG::forceNT;
    }
  }
  spl->s_iCnt++;
}

bool isRangeOutlier(rangeType rangeCurr, rangeType rangeLast, float coeff) {
  rangeType rangeDiff = rangeCurr - rangeLast;
  rangeType rangeThre = rangeCurr < rangeLast ? rangeCurr * coeff : rangeLast * coeff;
  if (rangeThre > g_sysConfig.s_outlierRngDiff)
    rangeThre = g_sysConfig.s_outlierRngDiff;
  if (rangeThre < g_sysConfig.s_fSampleDistence)
    rangeThre = g_sysConfig.s_fSampleDistence;
  if (rangeDiff > rangeThre || rangeDiff < -rangeThre)
    return true;
  return false;
}

TAILFLAG isTail(float tailCoeff, int interAngle, float *tailLUT, rangeType range) {
  int offset = interAngle << 1;
  int forceNtNum = floor(g_sysConfig.s_fSmallestMulti / range);
  if (forceNtNum < g_sysConfig.noTailMinSize)
    forceNtNum = g_sysConfig.noTailMinSize;
  if (forceNtNum >= g_sysConfig.s_iSampleNum)
    forceNtNum = g_sysConfig.s_iSampleNum;

  if (/* range > 2 &&  */ interAngle >= forceNtNum /* g_sysConfig.noTailMinSize */)
    return TAILFLAG::forceNT;
  if (offset > 60)
    return TAILFLAG::forceNT;
  if (tailCoeff < tailLUT[offset] || tailCoeff > tailLUT[offset + 1])
    return TAILFLAG::isT;
  return TAILFLAG::mayNT;
}

#ifdef _WIN32
#pragma endregion Sample
#endif

#ifdef _WIN32
#pragma region Segment
#endif

bool isMerge(s_Sample *spl, int newSecSta, OneSec *lastSec, int window) {
  return (!isRangeOutlier(spl->s_fRngs[newSecSta], spl->s_fRngs[lastSec->s_iSplEnd], 0.2)) && (newSecSta - lastSec->s_iSplEnd <= window);
}

void mergeSec(s_Sample *spl, Sections *secs, int newSta, int newEnd) {
  OneSec *rawSec = &secs->s_secs[secs->s_iCnt - 1];
  int lastSta = rawSec->s_iSplSta;
  int lastEnd = rawSec->s_iSplEnd;
  for (int m = lastEnd + 1; m <= newSta; ++m) {
    if (spl->s_bTailsCoeffs[m] == TAILFLAG::forceNT) {
      deleteSection(secs, -1);
      bool res1 = addSection(secs, spl, lastSta, m - 2);
      spl->s_bTails[m - 1] = false;
      spl->s_bTails[m] = false;
      bool res2 = addSection(secs, spl, m + 1, newEnd);
      if (res1 == false && res2 == false) {
      }
      lastSta = m + 1;
    }
  }
  deleteSection(secs, -1);
  addSection(secs, spl, lastSta, newEnd);
}

void segmentData(s_Sample *spl, rangeType *ranges, Sections *sec) {
  bool mayTail = false;
  int sectionSta = -1, sectionEnd = -1;
  bool beTail = false;
  int segmentIdDiff = g_sysConfig.noTailMinSize;
  if (segmentIdDiff < 1) {
    segmentIdDiff = 1;
  }
  int mergeWindow = 6;
  if (spl->s_iCnt <= 2) {
    for (int i = 0; i < spl->s_iCnt; ++i) {
      spl->s_bTails[i] = false;
    }
    return;
  }
  for (int i = 1; i < spl->s_iCnt; ++i) {
    beTail = checkTail(spl, i, 2);
    spl->s_bTails[i] = beTail;
    if (mayTail) {
      if (beTail) {
        if (isRangeOutlier(spl->s_fRngs[i], spl->s_fRngs[i - 1], 0.2) /* || ((spl->s_iIds[i] - spl->s_iIds[i - 1]) >= segmentIdDiff) */) {
          buildSection(sec, spl, sectionSta, sectionEnd, mergeWindow);
          sectionSta = i;
        }
        sectionEnd = i;
      } else {
        if (sectionSta != -1 && sectionEnd != -1) {
          buildSection(sec, spl, sectionSta, sectionEnd, mergeWindow);
        }
        sectionSta = -1;
        sectionEnd = -1;
        mayTail = false;
      }
    } else if (beTail) {
      mayTail = true;
      sectionSta = i;
      sectionEnd = i;
    }
  }

  spl->s_bTails[0] = spl->s_bTails[1];
  spl->s_bTails[spl->s_iCnt - 1] = spl->s_bTails[spl->s_iCnt - 2];
}

bool checkTail(s_Sample *spl, int id, int window) {
  bool frontTail = false, backTail = false;
  int idx = 0;
  // char allForce = 0;
  if (spl->s_bTailsCoeffs[id] == TAILFLAG::forceNT) {
    return false;
  }
  for (int i = 0; i < window; ++i) {
    idx = id - i;
    if (idx < 0) {
      break;
    }
    if (spl->s_bTailsCoeffs[idx] == TAILFLAG::forceNT) {
      break;
    }
    if (TAILFLAG::isT == spl->s_bTailsCoeffs[idx]) {
      frontTail = true;
      break;
    }
  }
  for (int i = 0; i < window; ++i) {
    idx = id + i + 1;
    if (idx >= spl->s_iCnt) {
      break;
    }
    if (spl->s_bTailsCoeffs[idx] == TAILFLAG::forceNT) {
      break;
    }
    if (TAILFLAG::isT == spl->s_bTailsCoeffs[idx]) {
      backTail = true;
      break;
    }
  }
  return frontTail && backTail;
}

void buildOneSec(OneSec *oneSec, s_Sample *spl, int sta, int end) {
  oneSec->s_iSplSta = sta;
  oneSec->s_iSplEnd = end;
  oneSec->s_iRawSta = spl->s_iIds[sta];
  oneSec->s_iRawEnd = spl->s_iIds[end];
  // oneSec->s_iPCnt = oneSec->s_iRawEnd - oneSec->s_iRawSta + 1;
  oneSec->s_iPCnt = end - sta + 1;
}

bool addSection(Sections *secs, s_Sample *spl, int sta, int end) {
  if (end - sta < 0)
    return false;
  int offset = secs->s_iCnt;
  buildOneSec(&secs->s_secs[offset], spl, sta, end);
  secs->s_iCnt++;
  return true;
}

void buildSection(Sections *secs, s_Sample *spl, int newSecSta, int newSecEnd, int mergeWindow) {
  bool checkMerge = false;
  OneSec *lastSec = &secs->s_secs[secs->s_iCnt - 1];
  if (secs->s_iCnt > 0) {
    checkMerge = isMerge(spl, newSecSta, lastSec, mergeWindow);
  }
  if (secs->s_iCnt > 0 && checkMerge) {
    mergeSec(spl, secs, newSecSta, newSecEnd);
  } else {
    addSection(secs, spl, newSecSta, newSecEnd);
  }
}

void deleteSection(Sections *secs, int id) {
  if (id == -1) {
    if (secs->s_iCnt > 0)
      secs->s_iCnt -= 1;
  }
}

void printSection(OneSec *sec, int idx) {
  if (printLevel < 0)
    return;

  std::cout << "sec [" << idx << "] | ";
  std::cout << "s_iSplSta: " << sec->s_iSplSta << " | ";
  std::cout << "s_iSplEnd: " << sec->s_iSplEnd << " | ";
  std::cout << "s_iRawSta: " << sec->s_iRawSta << " | ";
  std::cout << "s_iRawEnd: " << sec->s_iRawEnd << " | ";
  std::cout << "s_iPCnt: " << sec->s_iPCnt << std::endl;
}

#ifdef _WIN32
#pragma endregion Segment
#endif

#ifdef _WIN32
#pragma region AnaSection
#endif
void filter(Sections *sec, s_Sample *spl, rangeType *ranges) {
  for (int i = 0; i < sec->s_iCnt; ++i) {
    anaSection(sec, i, spl, ranges);
  }
}
int isFB(rangeType *ranges, int id) {
  if (id <= 0)
    return 1;
  if (id >= LEN)
    return 1;

  if ((ranges[id - 1] < g_sysConfig.s_fErrorCode || ranges[id] <= ranges[id - 1]) &&
      (ranges[id + 1] < g_sysConfig.s_fErrorCode || ranges[id] <= ranges[id + 1]))
    return 1;
  // if ((ranges[id - 1] < g_sysConfig.s_fErrorCode || ranges[id] >= ranges[id - 1]) &&
  //     (ranges[id + 1] < g_sysConfig.s_fErrorCode || ranges[id] >= ranges[id + 1]))
  //     return -1;
  return 0;
}
void findSectionSE(OneSec *sec, s_Sample *spl, rangeType *ranges, int *tailSecRawSta, int *tailSecRawEnd) {
  int splFG = -1;
  int splBG = -1;
  OneSec *oneSec = sec;
  int newSecRawSta = oneSec->s_iRawSta, newSecRawEnd = oneSec->s_iRawEnd;
  int staTmp = -1, endTmp = -1;

  findFBGround(oneSec, spl, ranges, &splFG, &splBG);
  if (splFG != -1) {
    findOutlier(ranges, spl->s_iIds[splFG], *tailSecRawSta, *tailSecRawEnd, &staTmp, &endTmp);
    if (endTmp != spl->s_iIds[splFG]) {
      newSecRawSta = endTmp;
    } else {
      if (splFG != oneSec->s_iSplSta) {
        for (int m = endTmp + 1; m <= oneSec->s_iRawSta; ++m) {
          if (ranges[m] > g_sysConfig.s_fErrorCode) {
            newSecRawSta = m;
            break;
          }
        }
      } else {
        newSecRawSta = oneSec->s_iRawSta;
      }
    }
  }
  if (splBG != -1) {
    findOutlier(ranges, spl->s_iIds[splBG], *tailSecRawSta, *tailSecRawEnd, &staTmp, &endTmp);
    if (staTmp != spl->s_iIds[splBG]) {
      newSecRawEnd = staTmp;
    } else {
      if (splBG != oneSec->s_iSplEnd) {
        for (int m = staTmp - 1; m >= oneSec->s_iRawEnd; m = m - 1) {
          if (ranges[m] > g_sysConfig.s_fRangeMin) {
            newSecRawEnd = m;
            break;
          }
        }
      } else {
        newSecRawEnd = oneSec->s_iRawEnd;
      }
    }
  }
  *tailSecRawEnd = newSecRawEnd;
  *tailSecRawSta = newSecRawSta;
}
void anaSection(Sections *sec, int id, s_Sample *spl, rangeType *ranges) {
  OneSec *oneSec = &sec->s_secs[id];
  int newSecRawSta = oneSec->s_iRawSta, newSecRawEnd = oneSec->s_iRawEnd;
  // int minNum = g_sysConfig.s_iTailMinNum;
  // int staTmp = -1, endTmp = -1;
  int lastPeakId = -1;
  // bool hasSep = false;
  printSection(oneSec, id);

  if (oneSec->s_iPCnt <= 2) {
    findSectionSE(oneSec, spl, ranges, &newSecRawSta, &newSecRawEnd);
    isSectionTail(ranges, newSecRawSta, newSecRawEnd);
  } else {
    OneSec newSec;
    lastPeakId = oneSec->s_iSplSta;
    for (int i = oneSec->s_iSplSta + 1; i <= oneSec->s_iSplEnd - 1; ++i) {
      rangeType rangeDiff_F = spl->s_fRngs[i] - spl->s_fRngs[i - 1];
      rangeType rangeDiff_B = spl->s_fRngs[i] - spl->s_fRngs[i + 1];
      if (spl->s_iIds[i] < newSecRawSta)
        continue;

      if (rangeDiff_F <= 0 && rangeDiff_B <= 0) {
        if (printLevel > 0)
          std::cout << "find F : " << i << std::endl;
        int tmpRawSta, tmpRawEnd;

        buildOneSec(&newSec, spl, lastPeakId, i);
        tmpRawSta = newSecRawSta;
        tmpRawEnd = newSecRawEnd;
        findSectionSE(&newSec, spl, ranges, &tmpRawSta, &tmpRawEnd);
        isSectionTail(ranges, tmpRawSta, tmpRawEnd);
        lastPeakId = i;
        // hasSep = true;
      } else if (rangeDiff_F >= 0 && rangeDiff_B >= 0) {
        if (printLevel > 0)

          std::cout << "find B : " << i << std::endl;
        // int tmpRawSta, tmpRawEnd;
        // tmpRawSta = newSecRawSta;
        // tmpRawEnd = newSecRawEnd;
        buildOneSec(&newSec, spl, lastPeakId, i);
        // findSectionSE(&newSec, spl, ranges, &tmpRawSta, &tmpRawEnd);
        // isSectionTail(ranges, tmpRawSta, tmpRawEnd);
        isSectionTail(ranges, newSec.s_iRawSta, newSec.s_iRawEnd);
        lastPeakId = i;
        // hasSep = true;
      }
    }
    if (lastPeakId != -1) {
      if (printLevel > 0)

        std::cout << "find F AND B END : " << lastPeakId << std::endl;

      // int splFG = -1;
      int tmpRawSta, tmpRawEnd;
      tmpRawSta = newSecRawSta;
      tmpRawEnd = newSecRawEnd;
      // int splBG = -1;
      buildOneSec(&newSec, spl, lastPeakId, oneSec->s_iSplEnd);
      findSectionSE(&newSec, spl, ranges, &tmpRawSta, &tmpRawEnd);
      isSectionTail(ranges, tmpRawSta, tmpRawEnd);
    } else {
      int tmpRawSta, tmpRawEnd;
      tmpRawSta = newSecRawSta;
      tmpRawEnd = newSecRawEnd;
      findSectionSE(oneSec, spl, ranges, &tmpRawSta, &tmpRawEnd);
      isSectionTail(ranges, tmpRawSta, tmpRawEnd);
    }
  }
}
void findOutlier(rangeType *ranges, int stdId, int startId, int endId, int *outId_F, int *outId_B) {
  rangeType rangeStd = ranges[stdId];
  rangeType distDiff = 0;
  rangeType rangeSave = g_sysConfig.s_fSaveRangeDiff;
  *outId_F = -1;
  *outId_B = -1;
  int lastId_F = stdId;
  int lastId_B = stdId;
  int saveCnt = g_sysConfig.s_iSaveMinNum;
  for (int i = stdId - 1; i >= startId; i = i - 1) {
    if ((ranges[i] < g_sysConfig.s_fRangeMin) || (ranges[i] > g_sysConfig.s_fRangeMax)) {
      continue;
    }
    distDiff = ranges[i] - rangeStd;
    if ((distDiff > rangeSave) || (distDiff < -rangeSave)) {
      *outId_F = i;
      break;
    }
    lastId_F = i;
  }
  for (int i = stdId + 1; i <= endId; ++i) {
    if ((ranges[i] < g_sysConfig.s_fRangeMin) || (ranges[i] > g_sysConfig.s_fRangeMax)) {
      continue;
    }
    distDiff = ranges[i] - rangeStd;
    if ((distDiff > rangeSave) || (distDiff < -rangeSave)) {
      *outId_B = i;
      break;
    }
    lastId_B = i;
  }
  if (lastId_B - lastId_F + 1 < saveCnt) {
    *outId_F = stdId;
    *outId_B = stdId;
  }
}

void isSectionTail(rangeType *ranges, int sta, int end) {
  if (end < sta)
    return;
  if (end == -1 || sta == -1)
    return;
  int num = end - sta + 1;
  if (num > g_sysConfig.s_iTailMaxNum)
    return;
  float l = 0;
  float angle = (end - sta) * ANGLE_RESO;
  rangeType rangeMin = 0;

  if (ranges[sta] < ranges[end]) {
    l = ranges[sta] * sin(angle);
    rangeMin = ranges[sta];
  } else {
    l = ranges[end] * sin(angle);

    rangeMin = ranges[end];
  }
  // if (l > 0.1 && rangeMin > 0.5)
  if (l > 0.15 && rangeMin > 0.5) {
    if (printLevel > 0)
      std::cout << "section is not Tail : [" << sta << "] : [" << end << "]: " << l << std::endl;
    return;
  }
  // else if (rangeMin <= 0.5)
  else if (rangeMin <= 0.5) {
    if (l > 1) {
      if (printLevel > 0)
        std::cout << "section is not Tail : [" << sta << "] : [" << end << "]: " << l << std::endl;
      return;
    }
  }
  if (printLevel > 0)
    std::cout << "section is Tail : [" << sta << "] : [" << end << "]: " << l << std::endl;
  for (int i = sta; i <= end; ++i) tailId[i] = true;
}
void findFBGround(OneSec *sec, s_Sample *spl, rangeType *ranges, int *sta, int *end) {
  int splSta = -1;
  int splEnd = -1;
  bool staIsFB = false;
  if (ranges[sec->s_iRawSta] <= ranges[sec->s_iRawEnd]) {
    staIsFB = true;
  }
  if (sec->s_iSplSta > 0) {
    if (false == spl->s_bTails[sec->s_iSplSta - 1] && staIsFB) {
      splSta = sec->s_iSplSta - 1;
      if (isRangeOutlier(spl->s_fRngs[sec->s_iSplSta], spl->s_fRngs[splSta], 0.2)) {
        splSta = sec->s_iSplSta;
      } else if (isFB(spl->s_fRngs, sec->s_iSplSta)) {
        splSta = sec->s_iSplSta;
      }
    } else {
      if (isFB(spl->s_fRngs, sec->s_iSplSta))
        splSta = sec->s_iSplSta;
    }
  }
  if (sec->s_iSplEnd < spl->s_iCnt - 1) {
    if (false == spl->s_bTails[sec->s_iSplEnd + 1] && (!staIsFB)) {
      splEnd = sec->s_iSplEnd + 1;
      if (isRangeOutlier(spl->s_fRngs[sec->s_iSplEnd], spl->s_fRngs[splEnd], 0.2)) {
        splEnd = sec->s_iSplEnd;
      } else if (isFB(spl->s_fRngs, sec->s_iSplEnd)) {
        splEnd = sec->s_iSplEnd;
      }
    } else {
      if (isFB(spl->s_fRngs, sec->s_iSplEnd))
        splEnd = sec->s_iSplEnd;
    }
  }
  *sta = splSta;
  *end = splEnd;
}

void filterFTail(rangeType *ranges) {
  int start = -1;
  int end = -1;
  int FTailMaxNum = 1000;
  int startIDs[1000];
  int endIDs[1000];
  int checkFTailId[1000];
  int checkCnt = 0;
  int minSize = 20;
  int tailWindow = 7;

  float checkRange = 2;
  int cnt = 0;
  for (int i = 1; i < LEN; ++i) {
    if (tailId[i] == false && ranges[i] > g_sysConfig.s_fErrorCode &&
        (ranges[i - 1] < g_sysConfig.s_fErrorCode || fabs(ranges[i] - ranges[i - 1]) < 0.3 /* 75 0.3 */)) {
      if (start == -1)
        start = i;
      end = i;
    } else {
      if (start != -1 && end != -1) {
        int saveNum = end - start + 1;
        if (saveNum < g_sysConfig.s_iSaveMinNum) {
          for (int j = start; j <= end; ++j)
            //     ranges[j] = 0;
            tailId[j] = true;
        } else if (g_sysConfig.FTail /*  && saveNum >= minSize */)  // if (saveNum >= 10)
        {
          if (printLevel > 0) {
            std::cout << "find no zero [ " << cnt << "]: " << start << " , " << end << std::endl;
          }
          startIDs[cnt] = start;
          endIDs[cnt] = end;
          if (cnt > 0) {
            float l = ranges[start] * sin((start - endIDs[cnt - 1]) * ANGLE_RESO);
            if ((/* (start - endIDs[cnt - 1] < 4) ||  */ (fabs(ranges[start] - ranges[endIDs[cnt - 1]]) < 0.05)) &&
                l < 0.02 /* (start - endIDs[cnt - 1]) < 10 */) {
              cnt -= 1;
              endIDs[cnt] = end;
            }
          }
          cnt++;
        }
      }
      if ((ranges[i] > g_sysConfig.s_fErrorCode) && (tailId[i] == false)) {
        start = i;
        end = i;
      } else {
        start = -1;
        end = -1;
      }
    }
  }
  for (int i = 0; i < cnt; ++i) {
    start = startIDs[i];
    end = endIDs[i];
    int saveNum = end - start + 1;
    if (saveNum < minSize)
      continue;
    if (ranges[start] < checkRange && checkCnt < FTailMaxNum) {
      checkFTailId[checkCnt++] = start;
      if (printLevel > 0)
        std::cout << "***** checkFTailIdF [ " << (checkCnt - 1) << "]: " << checkFTailId[checkCnt - 1] << std::endl;
    }
    if (ranges[end] < checkRange && checkCnt < FTailMaxNum) {
      checkFTailId[checkCnt++] = -end;
      if (printLevel > 0)
        std::cout << "***** checkFTailIdB [ " << (checkCnt - 1) << "]: " << checkFTailId[checkCnt - 1] << std::endl;
    }
  }
  int checkSta = 0, checkEnd = 0;
  if (g_sysConfig.FTail) {
    for (int i = 0; i < checkCnt; ++i) {
      if (checkFTailId[i] > 0) {
        checkSta = checkFTailId[i];
        checkEnd = checkSta + tailWindow;
      } else {
        checkSta = -checkFTailId[i] - tailWindow;
        checkEnd = -checkFTailId[i];
      }
      if (printLevel > 0)
        std::cout << "\n****** checkTail[" << i << "]: " << checkFTailId[i] << ", " << checkSta << " : " << checkEnd << std::endl;
      float l1;
      int noTailCnt = 0;

      if (checkFTailId[i] > 0) {
        for (int j = checkSta + 1; j <= checkEnd; ++j) {
          if (j >= LEN - 1)
            break;
          if (j < 1)
            continue;

          l1 = ranges[j] / ranges[j - 1];
          if (ranges[j - 1] < g_sysConfig.s_fRangeError) {
            continue;
          }
          if ((l1 < FTAIL_LUT[0] || l1 > FTAIL_LUT[1]) &&
              ((ranges[j] < g_sysConfig.s_fRangeError) || (fabs(ranges[j] - ranges[j - 1]) >= g_sysConfig.s_fFTailRangeError))) /* 5 0.02 */
          {
            tailId[j] = true;
            if (printLevel > 0)
              std::cout << " true Tail :" << j << std::endl;
            if (noTailCnt > 0 && noTailCnt < g_sysConfig.s_iSaveMinNum) {
              for (int m = j - noTailCnt; m < j; ++m) {
                if (printLevel > 0)
                  std::cout << " minSize Tail :" << m << std::endl;

                tailId[m] = true;
              }
            }
            noTailCnt = 0;
          } else {
            noTailCnt++;
          }
        }
        if (printLevel > 0)
          std::cout << " checkSta TailStaus :" << checkSta << ", " << tailId[checkSta] << ", " << tailId[checkSta + 1] << std::endl;

        tailId[checkSta] = tailId[checkSta + 1];
      } else {
        noTailCnt = g_sysConfig.s_iSaveMinNum;

        for (int j = checkSta + 1; j <= checkEnd; ++j) {
          if (j >= LEN - 1)
            break;
          if (j < 1)
            continue;

          l1 = ranges[j] / ranges[j - 1];
          if ((l1 < FTAIL_LUT[0] || l1 > FTAIL_LUT[1]) && (fabs(ranges[j] - ranges[j - 1]) >= g_sysConfig.s_fFTailRangeError /* 5 0.02 */)) {
            tailId[j] = true;
            if (printLevel > 0)
              std::cout << " true Tail :" << j << std::endl;

            if (noTailCnt > 0 && noTailCnt < g_sysConfig.s_iSaveMinNum) {
              for (int m = j - noTailCnt; m <= j; ++m) {
                if (printLevel > 0)
                  std::cout << " minSize Tail :" << m << std::endl;
                tailId[m] = true;
              }
            }
            noTailCnt = 0;
          } else {
            noTailCnt++;
          }
        }
        if (noTailCnt > 0 && noTailCnt < g_sysConfig.s_iSaveMinNum) {
          for (int m = checkEnd - noTailCnt; m <= checkEnd; ++m) {
            if (printLevel > 0)
              std::cout << " minSize Tail :" << m << std::endl;
            tailId[m] = true;
          }
        }
        if (printLevel > 0)
          std::cout << " checkEnd TailStaus :" << checkEnd << ", " << tailId[checkEnd] << ", " << tailId[checkEnd - 1] << std::endl;

        // tailId[checkEnd] = tailId[checkEnd - 1];
      }
    }
  }
}

void process(rangeType *ranges, int hz) {
  if (hz == 1) {
    /* 30hz */
    ANGLE_RESO = 0.00174f;
  } else if (hz == 0) {
    /* 10hz */
    ANGLE_RESO = 0.00058f;
  } else if (hz == 2) {
    /* 20hz */
    ANGLE_RESO = 0.00116f;
  }
  // ANGLE_RESO = 0.00116f;
  if (bNeedInit) {
    tfinit();
    bNeedInit = false;
  }
  printLevel = -1;
  sample(ranges, &g_splData);
  segmentData(&g_splData, ranges, &g_sec);
  filter(&g_sec, &g_splData, ranges);
  filterFTail(ranges);
  for (int i = 0; i < LEN; ++i) {
    if (tailId[i]) {
      ranges[i] = 0;
    }
  }
}

#ifdef _WIN32
#pragma endregion AnaSection
#endif
}  // namespace lidar
}  // namespace vanjee
