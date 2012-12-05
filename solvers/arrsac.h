#ifndef SOLVERS_ARRSAC_H_
#define SOLVERS_ARRSAC_H_

#include <algorithm>
#include <vector>

namespace solvers {
template<class Datum, class Model>
class Arrsac {
 public:
  Arrsac(int min_sample_size,
         double termination_inlier_ratio,
         int max_iters) {}
  ~Arrsac() {}

  bool Compute(const vector<Datum>& data,
               const Estimator<Datum, Model>& estimator,
               double error_thresh,
               Model* best_model);

 private:
  int GenerateInitialHypothesisSet();
};

template<class Datum, class Model>
int Arrsac::GenerateInitialHypothesisSet(vector<Model>* accepted_hypotheses) {
  int k = 1;
  int m_prime = m;
  int inner_ransac_its = 0;
  bool inner_ransac = false;
  int max_inner_ransac_its = 10000;
  int max_num_inliers = 0;

  // Vars to keep track of the avg inlier ratio of rejected hypotheses.
  int rejected_hypotheses = 0;
  double rejected_accum_inlier_ratio = 0;

  while (k <= m_prime) {
    Model hypothesis;
    if (!inner_ransac) {
      // generate hypothesis h(k) with k-th PROSAC sample
      hypothesis = GenerateHypothesisProsac();
    } else {
      // generate hypothesis h(k) with sampling subset from input
      hypothesis = GenerateHypothesisSampling();
      inner_ransac_its++;
      if (inner_ransac_its == max_inner_ransac_its) {
        inner_ransac_its = 0;
        inner_ransac = false;
      }
    }

    int observed_num_inliers;
    // evaluate hypothesis h(k) with SPRT
    bool sprt_test = SequentialProbabilityRatioTest(data,
                                                    hypothesis,
                                                    estimator,
                                                    error_thresh,
                                                    sigma,
                                                    epsilon,
                                                    decision_threshold,
                                                    &observed_num_inliers);
    // if hypothesis h(k) is rejected:
    if (!sprt_test) {
      // re-estimate params of SPRT (if required)
      // sigma = average of inlier ratios in bad models
      double observed_inlier_ratio = static_cast<double>(observed_num_inliers)/
          static_cast<double>(data.size());
      rejected_accum_inlier_ratio += observed_inlier_ratio;
      num_rejected_hypotheses++;
      sigma = rejected_accum_inlier_ratio/
          static_cast<double>(num_rejected_hypotheses);
    } else if (observed_num_inliers > max_num_inliers) {
      // else if hypothesis h(k) is accepted and has the largest support so far
      max_num_inliers = observed_num_inliers;
      //   inner_ransac = 1
      inner_ransac = true;

      //   inner_ransac_its = 0
      inner_ransac_its = 0;

      //   U_in = support of hypothesis h(k)

      // re-estimate params of SPRT
      // estimate epsilon as inlier ratio for largest size of support
      epsilon =
          static_cast<double>(max_num_inliers)/static_cast<double>(data.size());
      //   estimate inlier ratio e' and M_prime (eq 1) Cap M_prime at max of M
    }
    k = k+1;
  }
  return k;
}

template<class Datum, class Model>
void Arrsac::Compute(int max_candidate_hyps = 500,
                     int block_size = 100) {
  //   set parameters for SPRT test, calculate initial value of A
  double decision_threshold = CalculateSPRTDecisionThreshold(sprt_sigma,
                                                             sprt_epsilon);

  // Generate Initial Hypothesis Test
  vector<Model> hypotheses;
  int k = GenerateInitialHypothesisSet(&hypotheses);

  // Preemptive Evaluation
  for (int i = block_size+1; i < N; i++) {
    // Select n, the number of hypotheses to consider.
    int f_i = floor(max_candidate_hyps * pow(2, -1.0*floor(i/block_size)));
    int n = min(f_i, static_cast<double>(hypotheses.size())/2.0);

    // Rorder and select hypothesis h(1)...h(n)
    sort(hypothesis_score.begin(), hypothesis_score.end());
    // Resize to size n.
    
    if (n == 1) {
      break;
    }

    // score the hypotheses using data point i
    
    
    if (i % block_size == 0) {
      // Calculate best inlier ratio e' and num hypotheses M' (eq. 1)
      
      // M' = max(M,M')
      temp_max_candidate_hyps =
          max(max_candidate_hyps, temp_max_candidate_hyps);
      // if M' > k
      if (temp_max_candidate_hyps > k) {
        // Generate and evaluate M' - k new hypotheses on i data points
        // k = M'
        k = temp_max_candidate_hyps;
      }
    }
  }
  
  *model = hypotheses[0];
  return true;
}

}  // namespace solvers
#endif  // SOLVERS_ARRSAC_H_
