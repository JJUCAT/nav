#ifndef RECOVERY_EXCAPE__GENOTYPE_H_
#define RECOVERY_EXCAPE__GENOTYPE_H_

#include <vector>

namespace recovery_excape {

/**
 * @brief  基因定义了变量特征
 */
class Genotype
{
 public:

  Genotype();

  Genotype(const double geno, const double boundary_low,
    const double boundary_high, const double gen_width);

  ~Genotype();

  double Reproduce();

  void Mutation();

  double GetGeno() const;

 protected:

  double Random(const double min, const double max);

  double geno_;
  double boundary_low_;
  double boundary_high_; 
  double gen_width_;

}; // Genotype

using Chromosome = std::vector<Genotype>;

using HereditaryInformation = std::vector<Chromosome>;

} // namespace recovery_excape

#endif // RECOVERY_EXCAPE__GENOTYPE_H_
