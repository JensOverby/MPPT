static const int sz = 5;
static float median[sz];
static int m_i = 0;

bool medianFilter(float& val)
{
  median[m_i] = val;
  m_i++;
  if (m_i >= sz)
  {
    m_i = 0;
    float average = 0;
    for (int i=0; i<sz; i++)
      average += median[i];
    average /= sz;
    int j;
    float minval = 10000.;
    for (int i=0; i<sz; i++)
    {
      float d = fabs(median[i]-average);
      if (d < minval)
      {
        j=i;
        minval = d;
      }
    }
      
    val = median[j];
    return true;
  }
  return false;
}
