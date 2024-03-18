# ThirdParty-Mumps

This is an autotools-based build system to build and install
[MUltifrontal Massively Parallel sparse direct Solver](http://mumps.enseeiht.fr/) (MUMPS).
This installation of MUMPS is used by some other COIN-OR projects.

This version of ThirdParty-Mumps retrieves and builds MUMPS 5.5.0.

## Dependencies

- MUMPS source requires a Fortran 90 compiler.

- MUMPS requires LAPACK to be available. `configure` will look for a Lapack
  installation, but if that does not succeed, the flags to link with Lapack
  should be specified with the `--with-lapack-lflags` argument of `configure`.

- MUMPS can make use of METIS. `configure` will look for a METIS library and
  header, but if that does not succeed, the arguments `--with-metis-lflags`
  and `--with-metis-cflags` can be specified for `configure`.

  Both Metis 4 and Metis 5 can be used with ThirdParty-Mumps.

## Installation steps

1. Obtain the MUMPS source code.

   **********************************************************************
   Note: It is YOUR RESPONSIBILITY to ensure that you are entitled to
         download and use this third party package.
   **********************************************************************

   The shell script `get.Mumps` can be used to automatically download and
   extract the correct version of the MUMPS source code. The script will
   first try to download from from https://coin-or-tools.github.io/ThirdParty-Mumps.
   If this fails, it will attempt to download from the MUMPS website: http://mumps.enseeiht.fr/

   The script requires wget, curl, or fetch to be available on the system
   and in the shells search path (`$PATH`).

2. Run `./configure`. Use `./configure --help` to see available options.
   If using GFortran 10, add `ADD_FCFLAGS=-fallow-argument-mismatch` to
   your configure call (see above)

3. Run `make` to build the MUMPS library.

4. Run `make install` to install the MUMPS library and header files.

## NOTE for GFortran users

MUMPS source does not compile out of the box when using GFortran 10, probably
due to some incompatibilities between assumed Fortran language standards,
see also [issue #4](https://github.com/coin-or-tools/ThirdParty-Mumps/issues/4).

As a workaround, `configure` adds `-std=legacy` to the Fortran compiler flags
if `$FC` matches `*gfortran*`.

## Single-precision codes

This buildsystem can also build a single-precision version of MUMPS.
To do so, use the configure option `--with-precision=single`.

It is also possible to build both single- and double-precision variants
of MUMPS into the same library by using `--with-precision=all`.

## 64-bit integers

This buildsystem can be instructed to change the integer type of MUMPS to
have a size of 64-bit by using the configure option `--with-intsize=64`.
This defines `MUMPS_INTSIZE64` instead of `MUMPS_INTSIZE32` in
`mumps_int_def.h` and instructs the Fortran compiler to use 8-byte integers.
The latter requires that the Fortran compiler is either GNU Fortran or
Intel Fortran.

When MUMPS uses 64-bit integers, also Lapack and METIS need to use 64-bit
integers. For METIS, this means that `IDXTYPEWIDTH` needs to be set to 8
instead of 4.
