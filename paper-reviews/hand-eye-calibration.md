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

## 구현 코드 (C++)

이 코드는 Park와 Martin의 논문을 바탕으로 OpenCV를 사용하여 구현한 Hand-Eye Calibration 알고리즘이다.

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

## 핵심 알고리즘 분석

### 1. 큰 그림: AX = XB

손-눈 캘리브레이션(Hand-Eye Calibration)은 로봇 팔(손)과 카메라(눈)의 상대적 배치를 알아내는 문제다. 로봇 팔 끝에 카메라가 붙어있을 때, 로봇이 움직이면 카메라도 같이 움직인다. 이 두 움직임의 관계를 수학적으로 계산해서 "카메라가 로봇 팔에 대해 어떤 위치와 방향에 있는지"를 알아낸다.

$$AX = XB$$

- **A**: 로봇 팔의 움직임(변환 행렬)
- **B**: 카메라의 움직임(변환 행렬)
- **X**: 우리가 구하고자 하는 로봇 팔-카메라 사이의 관계 (회전 $R$과 이동 $t$)

이 코드는 여러 쌍의 로봇 움직임($H_g$)과 카메라 움직임($H_c$)을 입력으로 받아 $X$를 구한다. $X$는 회전 행렬 $R_{cam2gripper}$와 이동 벡터 $t_{cam2gripper}$로 나뉘어 계산된다.

### 2. 단계별 분석

문제를 회전($R$)과 이동($t$) 두 부분으로 나눠서 해결한다.

#### 2.1. 회전 부분 ($R$) 추정
여러 위치에서 찍힌 데이터를 가지고 두 위치 사이의 "차이"를 계산한다.

1. **상대적 움직임 계산**: $H_{gij} = H_g[j]^{-1} H_g[i]$, $H_{cij} = H_c[j] H_c[i]^{-1}$
2. **로드리게스 변환**: 3x3 회전 행렬을 "회전 축"과 "회전 각도"를 나타내는 3차원 벡터로 변환한다.
3. **행렬 M 축적**: 모든 데이터 쌍에 대해 $M += b \cdot a^T$를 수행한다.
4. **고유값 분해**: $M^T M$의 고유값과 고유벡터를 이용해 최종 회전 행렬 $R$을 도출한다.

#### 2.2. 이동 부분 ($t$) 추정
회전 $R$을 구했으니, 이제 위치 차이를 이용해 $t$를 구한다.

1. **선형 방정식 구성**: $(I - R_{gij}) \cdot t = t_{gij} - R \cdot t_{cij}$
2. **SVD(특이값 분해) 풀이**: 모든 쌍의 방정식을 쌓아서 $Ct = d$ 형태의 시스템을 만들고, SVD를 이용해 노이즈에 강인한 최소제곱해를 구한다.

## 쉬운 비유
로봇과 카메라가 친구라고 생각해보자. 로봇이 "왼쪽으로 2걸음, 회전 90도" 움직일 때 카메라도 비슷하게 움직이는데, 이 둘의 움직임 패턴을 보고 "카메라가 로봇의 어깨에 붙어있는지 머리에 붙어있는지" 알아내는 과정이다.

</div>
