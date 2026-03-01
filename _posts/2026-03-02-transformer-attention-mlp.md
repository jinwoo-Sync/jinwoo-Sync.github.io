---
layout: post
title: "Transformer의 두 엔진: Attention과 MLP는 각각 무슨 일을 하는가"
date: 2026-03-02
tags: [Deep Learning, Transformer, Attention, MLP, GPT]
---

## 동기

[MicroGPT 글](/2026/03/02/microgpt.html)에서 GPT의 전체 구조를 코드 수준으로 뜯어봤다. 그 과정에서 Transformer 블록 안에 **Attention**과 **MLP** 두 개의 서브 블록이 반복적으로 등장하는데, 각각이 정확히 어떤 역할을 하는지, 왜 둘 다 필요한지를 별도로 정리해본다.

---

## 1. Transformer 블록의 구조

![Transformer 블록 구조도]({{ '/assets/images/posts/transformer-attention-mlp/transformer_block.png' | relative_url }})

이 블록을 `n_layer`번 반복 적층한 것이 GPT다. GPT-2 small은 12번, GPT-3는 96번 쌓는다.

핵심은 **Attention과 MLP가 하는 일이 근본적으로 다르다**는 거다.

```
Attention = "시퀀스에서 어떤 토큰의 정보를 가져올까?"  (토큰 간 통신)
MLP       = "가져온 정보로 무슨 계산을 할까?"          (토큰 내부 처리)
```

---

## 2. 먼저 토큰이 뭔지

이 글에서 "토큰"이 계속 나오는데, NLP를 안 해봤으면 감이 안 올 수 있다.

토큰은 **텍스트를 잘게 쪼갠 조각**이다. 문자 하나일 수도 있고, 단어 하나일 수도 있고, 단어의 일부일 수도 있다.

```
문자 단위 토크나이저 (MicroGPT 방식):
"emma" → ["e", "m", "m", "a"]  → 토큰 4개

BPE 토크나이저 (GPT-4 방식):
"tokenization" → ["token", "ization"]  → 토큰 2개
```

Transformer 안에서 각 토큰은 **숫자 벡터 하나**로 표현된다. 예를 들어 `n_embd = 768`이면:

```
"고양이" → [0.12, -0.34, 0.56, ..., 0.78]   ← 768개 숫자로 된 벡터
"공원"   → [-0.45, 0.23, 0.11, ..., -0.67]  ← 또 다른 768차원 벡터
```

이 벡터 안에 해당 토큰의 의미 정보가 담겨 있다. Transformer는 이 벡터들을 입력으로 받아서 처리한다. 이걸 알면 아래 내용이 이해된다.

---

## 3. Attention: 토큰 간 정보 교환

### 3.1 Attention이 해결하는 문제

언어는 순서가 있고, 멀리 떨어진 단어들이 서로 관련된다.

```
"그 고양이는 어제 내가 공원에서 본 바로 그 고양이다"
 ↑                                        ↑
 이 두 "고양이"가 같은 대상임을 알아야 함
```

Attention 없이 MLP만으로는 이걸 처리할 수 없다. MLP는 각 토큰의 벡터를 **하나씩 독립적으로** 처리한다. "고양이" 벡터를 처리할 때 "공원" 벡터를 전혀 참조하지 않는다. 토큰 간 정보 교환이 일어나지 않는다.

### 2.2 Attention의 동작 방식

현재 토큰이 과거 토큰들을 "검색"하는 과정이다.

```python
# 각 토큰에서 Q, K, V 생성
q = x × Wq    # 현재 토큰: "나는 어떤 정보가 필요한가?"
k = x × Wk    # 각 토큰:   "나는 어떤 정보를 제공하는가?"
v = x × Wv    # 각 토큰:   "내 실제 내용은 이거다"

# 관련도 점수 계산
score[t] = q · k[t] / √d_k

# 확률로 변환
weights = softmax(scores)

# 가중합으로 정보 수집
output = Σ weights[t] × v[t]
```

직관적으로 보면:

```
현재 토큰 "고양이"의 Query가 과거 모든 토큰의 Key와 비교된다.

  토큰:     "그"   "고양이"  "는"   "어제"   "내가"  "공원에서"  "본"
  점수:     0.05    0.40    0.02    0.03     0.05     0.15     0.30
                     ↑                                          ↑
              같은 대상 = 높은 점수                    관련 동사 = 높은 점수
```

점수가 높은 토큰의 Value를 더 많이 가져온다. **"어디를 볼지"를 데이터가 결정한다**는 게 핵심이다.

### 2.3 Multi-Head: 여러 관점에서 동시에 보기

하나의 Attention head는 하나의 관점만 포착한다. 그래서 여러 head를 병렬로 둔다.

```
Head 0: 문법적 관계   → "고양이"와 "본"이 주어-술어 관계
Head 1: 의미적 유사성  → 첫 번째 "고양이"와 두 번째 "고양이"
Head 2: 위치적 근접성  → 바로 옆 토큰에 집중
Head 3: 장거리 의존성  → 문장 시작과 끝의 관계
```

실제로 학습된 GPT-2에서 이런 패턴이 관찰된다 (Vig & Belinkov, ACL 2019):

- Layer 0: 자기 자신이나 인접 토큰에 집중하는 헤드
- 중간 레이어: 문법적 의존 관계(주어-동사)를 가장 강하게 포착하는 헤드
- 깊은 레이어: 멀리 떨어진 토큰 간 관계를 보는 헤드

### 2.4 Attention이 못 하는 것

Attention은 **정보를 선택하고 모으는** 역할이다. 하지만 모은 정보를 **변환하거나 추론하는** 건 못 한다.

```
Attention 출력 = 0.4 × v["고양이"] + 0.3 × v["본"] + 0.15 × v["공원에서"] + ...
```

이건 그냥 기존 토큰 벡터들의 가중합이다. **새로운 표현을 만들어내는 게 아니라 기존 정보를 섞는 것**에 가깝다. 비선형 변환이 없기 때문에 표현력에 한계가 있다.

여기서 MLP가 필요해진다.

---

## 3. MLP: 토큰 내부 정보 처리

### 3.1 MLP의 구조

```python
x = rmsnorm(x)
x = linear(x, W1)        # n_embd → 4 × n_embd  (확장)
x = activation(x)        # 비선형 변환
x = linear(x, W2)        # 4 × n_embd → n_embd   (압축)
```

Attention이 "어떤 정보를 모았는지"를 입력으로 받아서, **비선형 변환을 통해 새로운 표현을 만들어낸다.**

### 3.2 왜 4배로 확장하는가

`n_embd = 768`이면 MLP 내부는 `3072`차원이 된다.

더 넓은 공간에서 작업하는 이유는, 저차원에서는 분리할 수 없는 패턴이 고차원에서는 분리 가능해지기 때문이다.

```
768차원:  복잡한 패턴이 뒤엉켜 있음
           ↓ W1 (확장)
3072차원: 패턴이 분리되어 각 뉴런이 독립적으로 활성화
           ↓ activation (비선형 변환)
3072차원: 관련 있는 뉴런만 살아남음
           ↓ W2 (압축)
768차원:  변환된 새로운 표현
```

### 3.3 MLP가 실제로 하는 일: 지식 저장소

2024년 이후의 연구들에서 MLP의 역할이 구체적으로 밝혀지고 있다.

**MLP는 "사실적 지식의 저장소"로 기능한다.** Attention이 맥락에서 관련 정보를 모아오면, MLP가 그 맥락에 맞는 지식을 활성화한다.

Meng et al. ("Locating and Editing Factual Associations in GPT", NeurIPS 2022)에서 이를 실험적으로 보였다:

```
입력: "The Eiffel Tower is located in"

Attention: "Eiffel Tower"와 "located in"의 관계를 파악
MLP:       "파리"라는 사실적 지식을 활성화 → 출력에 반영
```

특정 MLP 레이어의 가중치를 수정하면 **모델이 기억하는 사실을 직접 바꿀 수 있다.** "Eiffel Tower → Paris"를 "Eiffel Tower → London"으로 변경하는 것이 가능했다. 이건 MLP가 key-value 형태의 지식을 저장하고 있다는 강력한 증거다.

### 3.4 MLP의 뉴런 수준 분석

Geva et al. ("Transformer Feed-Forward Layers Are Key-Value Memories", EMNLP 2021)에서는 MLP를 이렇게 해석했다:

```
W1의 각 행 = "key" (어떤 패턴에 반응할지)
W2의 각 열 = "value" (반응했을 때 어떤 출력을 낼지)

W1[i] · x > threshold  →  W2[:, i]가 활성화
```

즉 MLP도 일종의 Attention과 유사한 key-value 구조이지만, **학습된 고정 패턴**에 대해 반응한다는 점에서 다르다. Attention은 입력 시퀀스에 따라 동적으로 달라지고, MLP는 학습된 지식에 대해 정적으로 반응한다.

---

## 4. Attention vs MLP: 역할 비교

| | Attention | MLP |
|---|---|---|
| **핵심 역할** | 토큰 간 정보 교환 | 토큰 내부 정보 변환 |
| **비유** | 회의에서 누구 말을 들을지 결정 | 들은 내용을 종합해서 결론 도출 |
| **입력 의존성** | 시퀀스 전체 (동적) | 현재 토큰만 (정적) |
| **연산 특성** | 선형 가중합 (softmax weighted sum) | 비선형 변환 (activation function) |
| **저장하는 것** | 없음 (매번 입력에서 계산) | 사실적 지식 (가중치에 인코딩) |
| **파라미터 비중** | ~1/3 | ~2/3 |

파라미터 비중이 흥미롭다. GPT 모델에서 **전체 파라미터의 약 2/3가 MLP**에 있다. Attention이 주목받지만, 실제로 모델의 "지식"은 대부분 MLP에 저장되어 있다.

---

## 5. 둘이 합쳐지면: Transformer 블록의 시너지

하나의 Transformer 블록이 처리하는 과정을 직관적으로 풀어보면:

```
입력: "에펠탑은 [  ]에 있다"의 빈칸을 채워야 함

[Attention 단계]
  현재 토큰 "[  ]"의 Query가 과거 토큰들의 Key를 검색
  → "에펠탑"과 "있다"에 높은 점수
  → 이 토큰들의 Value를 가중합
  → "에펠탑이라는 건물이 어딘가에 위치한다"는 맥락 정보 수집

[MLP 단계]
  Attention이 모아온 맥락 벡터를 입력으로 받음
  → W1으로 3072차원으로 확장
  → "에펠탑 + 위치" 패턴에 반응하는 뉴런들이 활성화
  → W2로 "파리"에 해당하는 방향의 벡터 출력
  → 최종적으로 "파리"가 다음 토큰 후보로 강화됨
```

**Attention이 관련 정보를 모아오고, MLP가 그 정보로 추론한다.** 이 두 단계가 레이어마다 반복되면서 점점 더 복잡한 추론이 가능해진다.

### 5.1 레이어가 깊어질수록

```
Layer 1-3:   저수준 패턴 (품사, 구문 구조)
Layer 4-8:   중간 수준 (의미적 관계, 상식)
Layer 9-12:  고수준 (추론, 사실 회상, 최종 예측)
```

얕은 레이어의 Attention은 문법적 관계를, 깊은 레이어의 Attention은 의미적 관계를 포착하는 경향이 있다. MLP도 마찬가지로 얕은 레이어는 저수준 특징을, 깊은 레이어는 사실적 지식을 활성화하는 경향이 있다.

---

## 6. 만약 하나만 있다면?

### Attention만 있으면

```python
# MLP 없이 Attention만
for layer in range(n_layer):
    x = attention(x)
    x = x + residual
```

토큰 간 정보는 교환되지만, **비선형 변환이 없어서 표현력이 제한**된다. 선형 연산의 반복은 결국 하나의 선형 변환과 동치이므로, 레이어를 아무리 쌓아도 복잡한 패턴을 학습할 수 없다.

### MLP만 있으면

```python
# Attention 없이 MLP만
for layer in range(n_layer):
    x = mlp(x)
    x = x + residual
```

각 토큰이 독립적으로 처리된다. **다른 토큰의 정보를 전혀 참조할 수 없다.** "에펠탑은 [  ]에 있다"에서 빈칸 토큰이 "에펠탑"이라는 토큰을 볼 수 없으니, 맥락에 맞는 답을 생성할 수 없다.

### 결론

**둘 다 필요하다.** Attention이 맥락을 모아오고, MLP가 그 맥락을 처리한다. 어느 하나가 빠지면 Transformer는 동작하지 않는다.

---

## 참고 논문

- Vaswani et al., "Attention Is All You Need", NeurIPS 2017 — Transformer 원 논문
- Geva et al., "Transformer Feed-Forward Layers Are Key-Value Memories", EMNLP 2021 — MLP가 key-value 메모리로 기능한다는 분석
- Meng et al., "Locating and Editing Factual Associations in GPT", NeurIPS 2022 — MLP의 특정 레이어가 사실적 지식을 저장한다는 실험적 증거
- Vig & Belinkov, "Analyzing the Structure of Attention in a Transformer Language Model", ACL 2019 — Attention head의 역할 분화 시각화
