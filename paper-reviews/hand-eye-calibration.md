---
layout: default
title: "Hand-Eye Calibration (AX=XB)"
paper: "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group"
authors: "F. Park, B. Martin"
venue: "IEEE Transactions on Robotics and Automation, 10(5): 717-721, 1994"
tags: [Calibration, Hand-Eye, Rodrigues, SVD]
---

<div class="container py-5">
  <nav aria-label="breadcrumb">
    <ol class="breadcrumb">
      <li class="breadcrumb-item"><a href="{{ '/' | relative_url }}">홈</a></li>
      <li class="breadcrumb-item"><a href="{{ '/papers.html' | relative_url }}">논문 리뷰</a></li>
      <li class="breadcrumb-item active">Hand-Eye Calibration</li>
    </ol>
  </nav>

  <h1 class="mb-3">Hand-Eye Calibration: AX=XB 문제</h1>
  <p class="text-muted">F. Park, B. Martin — IEEE Trans. Robotics and Automation, 1994</p>

  <hr>

<div class="content" markdown="1">

## 개요

손-눈 캘리브레이션(Hand-Eye Calibration)은 로봇 팔(손)과 카메라(눈)의 상대적 배치를 알아내는 문제다. 로봇 팔 끝에 카메라가 붙어있을 때, 로봇이 움직이면 카메라도 같이 움직인다. 이 두 움직임의 관계를 수학적으로 계산해서 "카메라가 로봇 팔에 대해 어떤 위치와 방향에 있는지"를 알아낸다.

$$AX = XB$$

- **A**: 로봇 팔의 움직임 (변환 행렬)
- **B**: 카메라의 움직임 (변환 행렬)
- **X**: 우리가 구하고자 하는 로봇 팔-카메라 사이의 관계 (회전 $R$과 이동 $t$)

이 코드는 여러 쌍의 로봇 움직임 $H_g$와 카메라 움직임 $H_c$를 입력으로 받아 $X$를 구한다. $X$는 회전 행렬 $R_{\text{cam2gripper}}$와 이동 벡터 $t_{\text{cam2gripper}}$로 나뉘어 계산된다.

---

## C++ 구현 코드 (OpenCV)

```cpp
static void calibrateHandEyePark(const std::vector<Mat>& Hg, const std::vector<Mat>& Hc,
                                 Mat& R_cam2gripper, Mat& t_cam2gripper)
{
    Mat M = Mat::zeros(3, 3, CV_64FC1);

    for (size_t i = 0; i < Hg.size(); i++)
    {
        for (size_t j = i+1; j < Hg.size(); j++)
        {
            Mat Hgij = homogeneousInverse(Hg[j]) * Hg[i];
            Mat Hcij = Hc[j] * homogeneousInverse(Hc[i]);

            Mat Rgij = Hgij(Rect(0, 0, 3, 3));
            Mat Rcij = Hcij(Rect(0, 0, 3, 3));

            Mat a, b;
            Rodrigues(Rgij, a);
            Rodrigues(Rcij, b);

            M += b * a.t();
        }
    }

    Mat eigenvalues, eigenvectors;
    eigen(M.t()*M, eigenvalues, eigenvectors);

    Mat v = Mat::zeros(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        v.at<double>(i,i) = 1.0 / sqrt(eigenvalues.at<double>(i,0));
    }

    Mat R = eigenvectors.t() * v * eigenvectors * M.t();
    R_cam2gripper = R;

    int K = static_cast<int>((Hg.size()*Hg.size() - Hg.size()) / 2.0);
    Mat C(3*K, 3, CV_64FC1);
    Mat d(3*K, 1, CV_64FC1);
    Mat I3 = Mat::eye(3, 3, CV_64FC1);

    int idx = 0;
    for (size_t i = 0; i < Hg.size(); i++)
    {
        for (size_t j = i+1; j < Hg.size(); j++, idx++)
        {
            Mat Hgij = homogeneousInverse(Hg[j]) * Hg[i];
            Mat Hcij = Hc[j] * homogeneousInverse(Hc[i]);

            Mat Rgij = Hgij(Rect(0, 0, 3, 3));
            Mat tgij = Hgij(Rect(3, 0, 1, 3));
            Mat tcij = Hcij(Rect(3, 0, 1, 3));

            Mat I_tgij = I3 - Rgij;
            I_tgij.copyTo(C(Rect(0, 3*idx, 3, 3)));

            Mat A_RB = tgij - R*tcij;
            A_RB.copyTo(d(Rect(0, 3*idx, 1, 3)));
        }
    }

    Mat t;
    solve(C, d, t, DECOMP_SVD);
    t_cam2gripper = t;
}
```

---

## 알고리즘 분석

문제를 **회전($R$)**과 **이동($t$)** 두 부분으로 나눠서 해결한다.

### 1단계: 회전 $R$ 추정

#### 1.1 상대적 움직임 계산

여러 위치에서 취득한 데이터를 가지고, 두 위치 사이의 "차이"를 계산한다.

$$H_{gij} = H_g[j]^{-1} \cdot H_g[i], \quad H_{cij} = H_c[j] \cdot H_c[i]^{-1}$$

- $H_{gij}$: 로봇이 위치 $j$에서 위치 $i$로 이동한 상대적 변화
- $H_{cij}$: 카메라가 위치 $j$에서 위치 $i$로 이동한 상대적 변화

#### 1.2 로드리게스 변환

4x4 동차 행렬에서 회전 부분(3x3)을 추출하고, `Rodrigues()` 함수로 3차원 벡터(회전축 + 회전각)로 변환한다.

- $a$: 로봇 회전 $R_{gij}$의 로드리게스 벡터
- $b$: 카메라 회전 $R_{cij}$의 로드리게스 벡터

#### 1.3 행렬 $M$ 축적

모든 데이터 쌍에 대해 외적 형태의 합을 구한다:

$$M = \sum_{(i,j)} b \cdot a^T$$

$M$은 로봇과 카메라 회전의 관계를 종합적으로 담은 행렬이다.

#### 1.4 고유값 분해로 $R$ 도출

$M^T M$의 고유값 분해를 수행하고, 고유값의 제곱근 역수를 대각 행렬 $v$에 배치한다:

$$R = V^T \cdot v \cdot V \cdot M^T$$

여기서 $V$는 고유벡터 행렬이다. 결과로 얻어지는 $R$이 $R_{\text{cam2gripper}}$, 즉 로봇과 카메라 사이의 회전 관계다.

---

### 2단계: 이동 $t$ 추정

회전 $R$을 구한 뒤, 위치 차이를 이용해 $t$를 구한다.

#### 2.1 선형 방정식 구성

각 데이터 쌍에 대해 다음 방정식을 세운다:

$$(I - R_{gij}) \cdot t = t_{gij} - R \cdot t_{cij}$$

$N$개 데이터에서 $K = N(N-1)/2$개 쌍을 만들어, 모든 방정식을 쌓으면 $Ct = d$ 형태의 대형 선형 시스템이 된다.

- $C$: $3K \times 3$ 계수 행렬
- $d$: $3K \times 1$ 상수항 벡터

#### 2.2 SVD 풀이

$$Ct = d$$

를 SVD(특이값 분해)로 풀어 노이즈에 강인한 최소제곱해를 구한다. 결과가 $t_{\text{cam2gripper}}$다.

---

## 요약

| 단계 | 입력 | 방법 | 출력 |
|:---:|:---:|:---:|:---:|
| 회전 추정 | $H_g$, $H_c$ | 로드리게스 변환 → 행렬 $M$ 축적 → 고유값 분해 | $R_{\text{cam2gripper}}$ |
| 이동 추정 | $R$, $H_g$, $H_c$ | 선형 방정식 구성 → SVD 풀이 | $t_{\text{cam2gripper}}$ |

</div>
</div>
