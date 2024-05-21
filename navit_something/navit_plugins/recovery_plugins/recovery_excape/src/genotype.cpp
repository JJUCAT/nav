#include <recovery_excape/genotype.h>
#include <random>

namespace recovery_excape {

Genotype::Genotype() {}

Genotype::~Genotype() {}

Genotype::Genotype(const double geno, const double boundary_low,
  const double boundary_high, const double gen_width) :
  geno_(geno), boundary_low_(boundary_low), boundary_high_(boundary_high), gen_width_(gen_width)
{}


double Genotype::Reproduce()
{
  double min = geno_ - gen_width_;
  min = std::max(min, boundary_low_);

  double max = geno_ + gen_width_;
  max = std::min(max, boundary_high_);

  double geno = Random(min, max);
  return geno;
}

void Genotype::Mutation()
{
  geno_ = Random(boundary_low_, boundary_high_);
}

double Genotype::GetGeno() const
{
  return geno_;
}



// -------------------- protected --------------------

double Genotype::Random(const double min, const double max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(min, max);
  double random = dist(gen);
  return random;
}



// -------------------- private --------------------



} // namespace recovery_excape
