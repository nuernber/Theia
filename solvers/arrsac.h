#ifndef SOLVERS_ARRSAC_H
#define SOLVERS_ARRSAC_H

void GenerateInitialHypothesisSet(vector<Model>* accepted_hypotheses) {
  int k = 1;
  int m_prime = m;
  int count_in = 0;
  int flag_in = 0;
  int max_inner_ransac_its = 10000;
  while (k > m_prime) {
    Model hypothesis;
    if (flag_in == 0) {
      // generate hypothesis h(k) with k-th PROSAC sample
      hypothesis = GenerateHypothesisProsac();
    } else {
      // generate hypothesis h(k) with sampling subset from input
      hypothesis = GenerateHypothesisSampling();
      count_in++;
      if (count_in == max_inner_ransac_its) {
        // reset count_in, flag_in
      }
    }

    // evaluate hypothesis h(k) with SPRT
    bool sprt_test = SPRT(hypothesis);
    // if hypothesis h(k) is rejected:
    if (!sprt_test) {
      // ???
      //   re-estimate params of SPRT (if required)
    }

    // else if hypothesis h(k) is accepted and has the largest support so far
    //   flag_in = 1
    //   count_in = 0
    //   U_in = support of hypothesis h(k)
    //   re-estimate params of SPRT
    //   estimate inlier ratio e' and M_prime (eq 1) Cap M_prime at max of M
    // k = k+1
  }
  // return k, set with accepted hypotheses
}

void compute() {
  // Initialize:
  //   set values for M and B
  int max_candidate_hyps = 500;
  int block_size = 100;
  //   set parameters for SPRT test, calculate initial value of A
  SetSPRTParameters();

  // Generate Initial Hypothesis Test
  // k = total number of hypotheses generated in initial stage
  int k = GenerateInitialHypothesisSet();

  // Preemptive Evaluation
  //   for i = B+1...N do:
  for (int i = block_size+1; i < N; i++) {
    //     Set p = num hypoth remaining

    //     n = min( f(i), p/2)
    int f_i = floor(max_candidate_hyps * pow(2, -1.0*floor(i/block_size)));
    int n = min(f_i, p/2);

    //     Rorder and select hypothesis h(1)...h(n)
    hypotheses.clear();
    hypotheses = GetTopKHypotheses(n);
    //     if n == 1
    //       break with remaining hypothesis as top
    if (n == 1) {
      break;
    }

    //   score the hypotheses using data point i

    //   if (i mod B) == 0
    if (i % block_size == 0) {
      //     Calculate best inlier ratio e' and num hypotheses M' (eq. 1)
      //     M' = max(M,M')
      temp_max_candidate_hyps =
          max(max_candidate_hyps, temp_max_candidate_hyps);
      //     if M' > k
      if (temp_max_candidate_hyps > k) {
        //       Generate and evaluate M' - k new hypotheses on i data points
        //       k = M'
        k = temp_max_candidate_hyps;
      }
    }
  }
}

#endif  // SOLVERS_ARRSAC_H
