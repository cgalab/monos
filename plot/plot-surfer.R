#!/usr/bin/env Rscript

library(ggplot2)
library(scales)
library(reshape2)
library(DBI)
library("optparse")


############################## functions #######################
# based on
# https://stackoverflow.com/questions/30179442/plotting-minor-breaks-on-a-log-scale-with-ggplot
log_breaks = function(maj, radix=10) {
  function(x) {
    minx         = floor(min(logb(x,radix), na.rm=T)) - 1
    maxx         = ceiling(max(logb(x,radix), na.rm=T)) + 1
    n_major      = maxx - minx + 1
    major_breaks = seq(minx, maxx, by=1)
    if (maj) {
      breaks = major_breaks
    } else {
      steps = logb(radix/2,radix)
      #steps = logb(1:(radix-1),radix)
      breaks = rep(steps, times=n_major) +
               rep(major_breaks, each=radix-1)
    }
    radix^breaks
  }
}
scale_x_log_eng = function(..., radix=10) {
  scale_x_continuous(...,
                     trans=log_trans(radix),
                     breaks=log_breaks(TRUE, radix),
                     minor_breaks=log_breaks(FALSE, radix),
                     labels = trans_format("log10", math_format(10^.x)),
                     )
}
scale_y_log_eng = function(..., radix=10) {
  scale_y_continuous(...,
                     trans=log_trans(radix),
                     breaks=log_breaks(TRUE, radix),
                     minor_breaks=log_breaks(FALSE, radix),
                     labels = trans_format("log10", math_format(10^.x)),
                     )
}
########################## end functions #######################
option_list = list(
  make_option(c("-c", "--color-dirs"), action="store_true", default=FALSE, help="Different color for different input directories"),
  make_option(c("-C", "--csv"), action="store_true", default=FALSE, help="Read csv instead of sqlite files"),
  make_option(c("-m", "--memory"), action="store_true", default=FALSE, help="Plot memory rather than runtime"),
  make_option(c("--compare-plot-1"), action="store_true", default=FALSE, help="Set up colors and shapes for the big compare-plot"),
  make_option(c("--compare-plot-2"), action="store_true", default=FALSE, help="Do selects for the type plot"),
  make_option(c("--compare-plot-3"), action="store_true", default=FALSE, help="Do monos v surfer"),
  make_option(c("--compare-plot-4"), action="store_true", default=FALSE, help="Do monos v monos (old cgal)"),
  make_option(c("--pdfsize"), default=1, type="integer", help="Predefined pdf sizes")
)

opt_parser = OptionParser(usage = "%prog [options] <pdffile> <dbfile> <dbfile..>", option_list=option_list)
parsed = parse_args(opt_parser, positional_arguments = c(2, Inf))
opt = parsed$options
arg = parsed$args

pdffile = arg[1]

dbnames = list()
for (dbname in arg) {
  dbnames = append(dbnames, dbname)
}
dbnames[1] = NULL

dfs = list()
if (opt$'csv') {
   for (dbname in dbnames) {
     csvdata = read.csv(dbname, header = F, na.strings = "")
     dbdata = data.frame(csvdata[1])
     dbdata['size'] = csvdata[1]
     if (opt$memory) {
       dbdata['value'] = csvdata[3]/1024
     } else {
       dbdata['value'] = csvdata[2]
     }
     dbdata['variable'] = dbname

     dfs = rbind(dfs, dbdata)
   }
} else {
   for (dbname in dbnames) {
     con = dbConnect(drv=RSQLite::SQLite(), dbname=dbname)
     # dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0 AND not input like '%octo%' AND not input like '%smr%'")
     # dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0 AND (input like '%/srpg_rnd/%' OR input like 'poly%')")
     # dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0 AND not input like '%srpg_iso%'")
     #dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0 AND input like 'poly-s%' OR input like '%/poly-s%'")
     if (opt$'compare-plot-2') {
       dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0 AND (
                      input like '%/srpg_rnd/%' OR
                      input like '%/srpg_octo/%' OR
                      input like '%/srpg_iso/%' OR
                      0 )")
     } else {
       dbdata = dbGetQuery( con, "SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0")
    }
     dbDisconnect(con)
     if (nrow(dbdata) == 0) {
       print(sprintf("No data loaded from %s", dbname))
       next
     }

     if (opt$memory) {
       colnames(dbdata)[ grep('memuse', colnames(dbdata)) ] = 'value'
     } else {
       colnames(dbdata)[ grep('runtime', colnames(dbdata)) ] = 'value'
     }
     if (opt$'color-dirs') {
       dbdata['input'] = apply(dbdata['input'], 1, dirname)
       dbdata['input'] = apply(dbdata['input'], 1, basename)
       colnames(dbdata)[ grep('input', colnames(dbdata)) ] = 'variable'
     } else {
       dbdata['variable'] = dbname
     }

     dfs = rbind(dfs, dbdata)
   }
}


if (opt$memory) {
  ylimits = c(100, 10000)
} else {
  if (opt$'compare-plot-2') {
    ylimits = c(0.01, 1000)
  } else if (opt$'compare-plot-3') {
    ylimits = c(1, 10000)
  } else if (opt$'compare-plot-4') {
    ylimits = c(.3, 100)
  } else {
    ylimits = c(0.001, 1000)
  }
}
if (opt$'compare-plot-3') {
  xlimits = c(10000, 1000000)
} else if (opt$'compare-plot-4') {
  xlimits = c(1000, 100000)
} else {
  xlimits = c(100, 100000)
}

if (opt$'compare-plot-1') {
  sp = ggplot(dfs, aes(x=size, y=value, color=variable, shape=variable))
  colors = c(
    'cgal-ss-interior.db' = '#ab0009',
    'monos.db' = '#00ab2e',
    'surfer-core.db' = '#c27dff',
    'surfer-core-interior.db' = '#006f91',
    'surfer-double.db' = '#35cf8f'
  )
  # doesn't do anything
  alphas = c(
    'cgal-ss-interior.db' = 1,
    'monos.db' = 1,
    'surfer-core.db' = 0.9,
    'surfer-core-interior.db' = 0.9,
    'surfer-double.db' = 1
  )
  shapes = c(
    'cgal-ss-interior.db' = 15,
    'monos.db' = 4,
    'surfer-core.db' = 3,
    'surfer-core-interior.db' = 16,
    'surfer-double.db' = 17
  )
  labels = c(
    'cgal-ss-interior.db' = 'CGAL (interior-only)',
    'monos.db' = 'Monos',
    'surfer-core.db' = 'Surfer (CGAL::Expr, plane)',
    'surfer-core-interior.db' = 'Surfer (CGAL::Expr, interior-only)',
    'surfer-double.db' = 'Surfer (IEEE 754, plane)'
  )
  sp = sp + scale_color_manual(values = colors, labels=labels)
  # sp = sp + scale_alpha_manual(values = alphas)
  sp = sp + scale_shape_manual(values = shapes, labels=labels)
  sp = sp + geom_point(size=1, na.rm=TRUE)
} else if (opt$'compare-plot-2') {
  sp = ggplot(dfs, aes(x=size, y=value, color=variable, shape=variable))
  colors = c(
    'srpg_iso' = '#ab0009',
    'srpg_octo' = '#00ab2e',
    'srpg_rnd' = '#000dff'
  )
  shapes = c(
    'srpg_iso' = 15,
    'srpg_octo' = 16,
    'srpg_rnd' = 17
  )
  sp = sp + scale_color_manual(values = colors)
  sp = sp + scale_shape_manual(values = shapes)
  sp = sp + geom_point(size=0.4, na.rm=TRUE)
} else if (opt$'compare-plot-3') {
  sp = ggplot(dfs, aes(x=size, y=value, color=variable, shape=variable))
  colors = c(
    'monos-v-surfer/monos.db' = '#00ab30',
    'monos-v-surfer/surfer-monotone.db' = '#006f91'
  )
  shapes = c(
    'monos-v-surfer/monos.db' = 4,
    'monos-v-surfer/surfer-monotone.db' = 16
  )
  labels = c(
    'monos-v-surfer/monos.db' = 'Monos',
    'monos-v-surfer/surfer-monotone.db' = 'Surfer (CGAL::Expr, interior-only)'
  )
  sp = sp + scale_color_manual(values = colors, labels=labels)
  sp = sp + scale_shape_manual(values = shapes, labels=labels)
  sp = sp + geom_point(size=0.4, na.rm=TRUE)
} else if (opt$'compare-plot-4') {
  sp = ggplot(dfs, aes(x=size, y=value, color=variable, shape=variable))
  colors = c(
    'monos-cgal/monos.db' = '#00ab30',
    'monos-cgal/monos-oldcgal.db' = '#006f91'
  )
  shapes = c(
    'monos-cgal/monos.db' = 4,
    'monos-cgal/monos-oldcgal.db' = 16
  )
  labels = c(
    'monos-cgal/monos.db' = 'Monos CGAL 5.0',
    'monos-cgal/monos-oldcgal.db' = 'Monos CGAL 4.13'
  )
  sp = sp + scale_color_manual(values = colors, labels=labels)
  sp = sp + scale_shape_manual(values = shapes, labels=labels)
  sp = sp + geom_point(size=0.4, na.rm=TRUE)
} else {
  sp = ggplot(dfs, aes(x=size, y=value, color=variable))
  sp = sp + geom_point(size=0.4, na.rm=TRUE)
}
sp = sp +
      #scale_x_log10(breaks = trans_breaks("log10", function(x) 10^x),
      #              #labels = trans_format("log10", math_format(10^.x)),
      #              limits = xlimits,
      #              ) +
      #scale_y_log10(breaks = trans_breaks("log10", function(x) 10^x),
      #              labels = trans_format("log10", math_format(10^.x))
      #              ,limits = ylimits
      #              )
      scale_x_log_eng(
        limits = xlimits,
        ) +
      scale_y_log_eng(
        limits = ylimits,
        )
if (opt$'pdfsize' == 2) {
  sp = sp + annotation_logticks(
    short = unit(0.25,"mm"),
    mid = unit(0.7,"mm"),
    long = unit(1,"mm"),
    sides = 'trbl',
    )
} else {
  sp = sp + annotation_logticks(
    short = unit(1,"mm"),
    mid = unit(3,"mm"),
    long = unit(4,"mm"),
    sides = 'trbl',
    )
}

sp = sp + theme_bw()
if (opt$memory) {
  sp = sp + ylab("Memory Use [MiB]")
} else {
  sp = sp + ylab("Runtime [s]")
}
sp = sp + xlab("# Vertices")

sp = sp + theme(
                 legend.key = element_blank(),
                 # legend.position="right",
                 legend.title = element_blank(),
                 legend.key.size = unit(0.35, "cm"),
                 legend.key.width = unit(0.35,"cm")
                )

if (opt$'pdfsize' == 2) {
  sp = sp + theme(
                 text = element_text(size=7),
                 axis.title.x = element_text(margin = margin(t = 0, r = 0, b = -6, l = 0)),
                 axis.title.y = element_text(margin = margin(t = 0, r = 0, b = 0, l = -4)),
                 panel.grid.major.x = element_line(size=0.25),
                 panel.grid.minor.x = element_line(size=0),
                 panel.grid.major.y = element_line(size=0.25),
                 panel.grid.minor.y = element_line(size=0)
                 )

  if (opt$'compare-plot-3') {
    sp = sp + theme(
                   legend.text = element_text(size=5),
                   legend.position = c(0.65, 0.2)
                   )
  } else {
    sp = sp + theme(
                   legend.text = element_text(size=6),
                   legend.position = c(0.8, 0.2)
                   )
  }

  pdf(pdffile, width = 2.5, height = 2)
} else {
  sp = sp + theme(
                 legend.text = element_text(size=10),
                 panel.grid.major.x = element_line(size=1),
                 panel.grid.minor.x = element_line(size=0.25),
                 panel.grid.major.y = element_line(size=1),
                 panel.grid.minor.y = element_line(size=0.25),
  )
  pdf(pdffile, width = 12, height = 10)
}
sp
graphics.off()
