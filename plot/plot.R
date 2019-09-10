library(ggplot2)
library(scales)
library(reshape2)

# NOT WOKRING -- 
##library(rstudioapi)
##setwd(dirname(rstudioapi::callFun("getActiveDocumentContext")$path)) 

setwd(dir="~/devel/ord/monos/plot/")

monos.pao <- read.csv("pao-mono-pnts.csv", header=F, na.strings="")

# rename column names
colnames(monos.pao)[1] <- "SIZE"
colnames(monos.pao)[2] <- "TIME"
colnames(monos.pao)[3] <- "Filename"

df <- monos.pao

#df <- merge(fist.regular,fist.regularcdt,by="FILENAME")
#df = merge(df,fist.triangle,by="FILENAME")

# remove not numerics of Tri field
# df <- df[!is.na(as.numeric(as.character(df$FISTDC8))), ] 
# data.all <- data.all[!is.na(as.numeric(as.character(data.all$FISTDC4))), ] 
# data.all <- data.all[!is.na(as.numeric(as.character(data.all$FISTDC8))), ] 
# data.all <- data.all[!is.na(as.numeric(as.character(data.all$FISTDC16))), ] 
# data.all <- data.all[!is.na(as.numeric(as.character(data.all$FISTDC32))), ] 
# data.all <- data.all[!is.na(as.numeric(as.character(data.all$FISTDC64))), ] 

# remove NSs produced by MS (debug output error)
df <- df[complete.cases(df),]

df <- df[order(df$SIZE),]

# calculate speedup
#df$SpeedupDC2 <- as.double(df$FIST / (df$FISTDC2))
#df$CpuInSecTRIANGLE <- as.double(df$TRIANGLE) / 1000.0
# data.all[(data.all$TriCount > 100 && data.all$Speedup32 <= 1), ]

# remove INF and NA rows
# data.all <- do.call(data.frame,lapply(data.all, function(x) replace(x, is.infinite(x),NA)))
# data.all <- data.all[complete.cases(data.all),]

# sort
# data.all <- data.all[order(data.all$TriCount,data.all$Speedup2),]
# data.all <- data.all[order(data.all$TriCount,data.all$Speedup4),]


# FILTER 
# filtered <- data.all[(data.all$Speedup32<0.8),]
# filtered <- filtered[(filtered$TriCount>100000),]
# filtered <- filtered[(filtered$TriCount<500000),]
# END FILTER


fancy_scientific <- function(l) {
  # turn in to character string in scientific notation
  l <- format(l, scientific = TRUE)
  # quote the part before the exponent to keep all the digits
  #l <- gsub("^(.*)e", "'\\1'e", l)
  l <- gsub("^(.*)e", "e", l)
  l <- gsub("\\+", "", l)
  # turn the 'e+' into plotmath format
  #l <- gsub("e", "%*%10^", l)
  l <- gsub("e", "10^", l)
  # return this as an expression
  parse(text=l)
}




# points 
pdf("monos-pao-mono.pdf", width = 3.7, height = 3.7)
ggplot(data = df, aes(x = SIZE)) +
  #  geom_point(aes(y = SpeedupDC16, colour = "FIST D&C 16   "), fill=NA, size=.7, alpha=.7) +
  geom_point(aes(y = TIME, colour = "Monos   "), fill=NA, size=.7, alpha=.7) +
  #geom_point(aes(y = SpeedupDC4, colour = "FIST D&C 4   "), fill=NA, size=.7, alpha=.7) +
  #geom_point(aes(y = SpeedupDC2, colour = "FIST D&C 2   "), fill=NA, size=.7, alpha=.7) +
  geom_line(y=1) +  
  scale_colour_manual("", 
                      breaks = c("Monos ", "FIST D&C 2   ", "FIST D&C 4   ", "FIST D&C 8   ", "FIST D&C 16   "
                                 , "FIST(P&C 32)   ", "FIST(P&C 64)   "),
                      values = c("green", "blue", "red", "magenta", "orange", "black")) +
  xlab("# vertices") +
  ylab("time [ms]") + 
  coord_cartesian(xlim=c(1,400000), ylim=c(0, 60000)) +
  guides(colour = guide_legend(override.aes = list(size = 5, alpha=1)))+
  theme_bw() + 
  theme(#plot.background = element_blank(),panel.grid.major = element_blank(),
    #panel.grid.minor = element_blank(),panel.border = element_blank(),    
    legend.key = element_blank(),
    legend.position="bottom"
    #legend.key=element_rect(fill=NA)
    #     legend.position="none"
  ) +
  scale_x_log10(labels = comma)
#   scale_x_continuous(labels = comma)  
dev.off()


