Surface fairing {#surf-fairing}
===============================

Fairing algorithm is implemented by paper \ref kallay93 "[Kallay, 1993]".
The fairing operator performs renumbering of control points to deduce serial
indices from the grid nodes. The following figure illustrates the
correspondence between the new indices \f$k\f$ and the initial indices \f$(i,j)\f$.

\image html bsurf-fairing-numbering.png
\image latex bsurf-fairing-numbering.png

Such renumbering allows rewriting equation of the initial B-surface
\f$\textbf{s}_0(u,v)\f$ in the following form:

\f[
  \textbf{s}_0(u,v) = \sum_i \sum_j \textbf{P}_{ij} N_i^p(u) N_j^q(v) = \sum_k \textbf{C}_k N_k(u,v)
\f]

Here \f$\textbf{C}_k\f$ are the original control points with serial numbering.
The new functions \f$N_k(u,v)\f$ correspond to the products of the original
B-spline basis functions.

\image html bsurf-fairing-operator.png
\image latex bsurf-fairing-operator.png
