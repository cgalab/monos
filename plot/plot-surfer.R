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
  make_option(c("-m", "--memory"), action="store_true", default=FALSE, help="Plot memory rather than runtime")
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
     dbdata = dbGetQuery( con, 'SELECT size, runtime, memuse / 1024, input FROM run WHERE exit = 0')

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
  ylimits = c(0.001, 1000)
}
xlimits = c(100, 100000)

sp = ggplot(dfs, aes(x=size, y=value, colour=variable))
sp = sp + geom_point(size=0.4)
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
sp = sp + annotation_logticks(
  short = unit(1,"mm"),
  mid = unit(3,"mm"),
  long = unit(4,"mm")
  )
sp = sp + theme_bw()
if (opt$memory) {
  sp = sp + ylab("Memory Use [MiB]")
} else {
  sp = sp + ylab("Runtime [s]")
}
sp = sp + xlab("# Vertices")

sp = sp + theme(
                 panel.grid.major.x = element_line(size=1),
                 panel.grid.minor.x = element_line(size=0.25),
                 panel.grid.major.y = element_line(size=1),
                 panel.grid.minor.y = element_line(size=0.25),
                 legend.key = element_blank(),
                 legend.position="bottom",
                 legend.title = element_blank(),
                 legend.text = element_text(size=10),
                 legend.key.size = unit(0.35, "cm"),
                 legend.key.width = unit(0.35,"cm")
                )

pdf(pdffile, width = 12, height = 10)
sp
dev.off()
